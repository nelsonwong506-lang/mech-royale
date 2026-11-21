#include "arena.hpp"
#include "navigation.hpp"
#include "survival.hpp"
#include "tactics.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <unistd.h>
#include <utility>
#include <vector>

namespace mr {
namespace {

using namespace std::chrono_literals;
constexpr double infinity = std::numeric_limits<double>::infinity();
constexpr std::array angles{0, 30, -30, 60, -60, 90, -90, 135, -135, 180};

struct Command {
    Vec2 move{};
    bool boost = false;
    bool jump = false;
};

struct Progress {
    Vec3 target;
    Vec3 origin;
    double time;
};

struct RetreatIntent {
    Vec3 target;
    std::optional<int> opponent;
    double expires;
};

struct Peek {
    Vec3 cover;
    Vec3 edge;
    int opponent;
    double started;
    bool returning = false;
    bool fired = false;
};

struct Decision {
    Command command;
    const char* label;
    bool fireAllowed = true;
    bool returningToCover = false;
};

struct ActiveHop {
    DefensiveHop path;
    double started;
    const char* label;
};

Vec2 sanitizeMove(Vec2 move) {
    move[0] = finiteOr(move[0], 0.0);
    move[1] = finiteOr(move[1], 0.0);
    const double norm = std::hypot(move[0], move[1]);
    if (norm > 1.0) {
        move[0] /= norm;
        move[1] /= norm;
    }
    return move;
}

Vec2 sanitizeAim(Vec2 aim) {
    return {finiteOr(std::remainder(aim[0], 2 * std::numbers::pi), 0.0),
            std::clamp(finiteOr(aim[1], 0.0), -1.50, 1.50)};
}

std::string toLowerText(const Json& item) {
    std::string raw = item.is_string() ? item.get<std::string>() : item.dump();
    for (char& ch : raw) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return raw;
}

int weaponPriority(std::string_view kind) {
    if (kind == "lightning") return 100;
    if (kind == "laser") return 75;
    if (kind == "mortar") return 55;
    if (kind == "gun") return 10;
    return 0;
}

int upgradePriority(const Json& choice) {
    if (choice.is_object()) {
        const auto idIt = choice.find("id");
        if (idIt != choice.end() && idIt->is_string()) {
            const std::string_view id = idIt->get_ref<const std::string&>();
            if (id == "nanites") return 100;
            if (id == "armor") return 96;
            if (id == "plating") return 92;
            if (id == "lightning_efficiency") return 90;
            if (id == "lightning_chain") return 88;
            if (id == "lightning_range") return 86;
            if (id == "gun_damage") return 84;
            if (id == "gun_rate") return 82;
            if (id == "laser_charge") return 80;
            if (id == "laser_damage") return 79;
            if (id == "gun_mag") return 78;
            if (id == "servos") return 76;
            if (id == "laser_pierce") return 72;
            if (id == "reactor") return 70;
            if (id == "jets") return 65;
            if (id == "mortar_reload") return 64;
            if (id == "capacitor") return 62;
            if (id == "mortar_splash") return 62;
            if (id == "mortar_velocity") return 58;
            if (id == "gun_velocity") return 56;
            if (id == "gyros") return 52;
            if (id == "sensors") return 48;
        }
    }
    const std::string desc = toLowerText(choice);
    int score = 0;
    if (desc.find("nanite") != std::string::npos || desc.find("regen") != std::string::npos) score += 50;
    if (desc.find("hp") != std::string::npos || desc.find("health") != std::string::npos ||
        desc.find("armor") != std::string::npos || desc.find("plating") != std::string::npos) score += 40;
    if (desc.find("lightning") != std::string::npos || desc.find("damage") != std::string::npos ||
        desc.find("fire rate") != std::string::npos || desc.find("gun") != std::string::npos) score += 35;
    if (desc.find("speed") != std::string::npos || desc.find("reload") != std::string::npos) score += 20;
    return score;
}

int chooseOfferPick(const Json& offer) {
    const Json* choices = nullptr;
    if (offer.is_array() && !offer.empty()) {
        choices = &offer;
    } else if (offer.is_object()) {
        for (const char* key : {"choices", "options", "items", "upgrades"}) {
            const auto it = offer.find(key);
            if (it != offer.end() && it->is_array() && !it->empty()) {
                choices = &(*it);
                break;
            }
        }
    }
    if (!choices || choices->empty()) {
        return 0;
    }
    int bestIndex = 0;
    int bestScore = std::numeric_limits<int>::min();
    for (std::size_t i = 0; i < choices->size(); ++i) {
        const int score = upgradePriority((*choices)[i]);
        if (score > bestScore) {
            bestScore = score;
            bestIndex = static_cast<int>(i);
        }
    }
    return bestIndex;
}

class Bot {
public:
    explicit Bot(const Json& hello)
        : config_(hello.is_object() ? hello.value("config", Json::object()) : Json::object()),
          arena_(hello.at("map").at("dir").get<std::string>(),
                 hello.at("map").value("files", std::vector<std::string>{})),
          navigator_(arena_), threats_(config_),
          pickupRadius_(config_.value("cubes", Json::object()).value("pickup_radius", 3.5)),
          lootPickupRadius_(config_.value("loot", Json::object()).value("pickup_radius", 4.0)),
          cubeStatCap_(config_.value("cubes", Json::object()).value("stat_cap", 10.0)) {}

    Json act(const Json& obs, Clock::time_point tickStarted) {
        const auto hardDeadline = tickStarted + 30ms;
        const auto& me = obs.contains("self") && obs["self"].is_object() ? obs["self"] : Json::object();
        const auto start = safeVec3(me, "pos");
        const double now = finiteOr(obs.value("t", obs.value("tick", 0.0) / 20.0), 0.0);
        const Zone zone(obs.contains("zone") && obs["zone"].is_object() ? obs["zone"] : Json::object(), now);
        auto stats = config_.value("mech", Json::object());
        if (me.contains("stats") && me["stats"].is_object()) {
            stats.update(me["stats"]);
        }
        const Motion motion(stats);
        const double muzzleHeight = stats.value("muzzle_height", 7.0);

        const auto threat = threats_.update(obs, arena_, std::min(hardDeadline, Clock::now() + 7ms));
        const auto projDodge = threats_.projectile_dodge(obs, arena_);
        const auto soundAlert = threats_.sound_alert(now);
        const bool underThreat = threat && (threat->hostile || threat->damaged_recently || threat->laser_charging);
        const auto duel = !underThreat
            ? threats_.duel_target(obs, arena_, std::min(hardDeadline, Clock::now() + 3ms))
            : std::nullopt;
        const auto enemyInfos = threats_.visible_enemies(obs, arena_);
        const auto enemies = localEnemies(obs, start, threat, motion.height);

        const bool inZone = me.value("in_zone", false);
        const RooftopNode* currentRoof = arena_.roofAt(start);
        const bool onRoof = currentRoof != nullptr || start[2] > arena_.ground(start[0], start[1]) + 4.5;
        const double hp = me.value("hp", 1400.0);
        const double maxHp = std::max(1.0, me.value("max_hp", 1400.0));
        const double hpFrac = hp / maxHp;
        const bool hasNanites = stats.value("regen_delay", 5.0) < 4.5;
        const auto window = rotationWindow(start, zone, motion, routeEstimate_);

        int exposedNow = 0;
        int close80 = 0;
        int close70 = 0;
        bool enemyOpening = false;
        for (const auto& e : enemyInfos) {
            const double d = distanceXY(start, e.pos);
            if (d <= 80.0) ++close80;
            if (d <= 70.0) ++close70;
            if (d <= 185.0 && wallCover(start, e.pos, motion.height, arena_) < 3) {
                ++exposedNow;
            }
            if (d <= 75.0 && !e.facing_us && !e.charging && e.hp <= 560.0 && hpFrac >= 0.58) {
                enemyOpening = true;
            }
        }
        if (close80 > 1) {
            enemyOpening = false;
        }

        // Dynamic Fight-First + Regen-to-100% Rule:
        // Never abandon a finishable kill or winning 1v1; once safely regenerating, heal to ~100% (98%)
        // unless zone urgency, finishable kill, or clear enemy opening!
        const bool finishableClose = threat && threat->position && threat->hp <= 540.0 &&
            distanceXY(start, *threat->position) <= 75.0 && hpFrac >= 0.22;
        const bool winning1v1 = close80 <= 1 && threat && threat->position &&
            threat->hp < hp * 0.85 && hpFrac >= 0.25;
        if (!inZone || window.urgent || hpFrac >= 0.98 || finishableClose || winning1v1 || enemyOpening) {
            regenMode_ = false;
        } else if (hpFrac < 0.42 || (hpFrac < 0.68 && (onRoof || exposedNow == 0 || hasNanites))) {
            regenMode_ = true;
        }

        // Track anchor staleness to encourage frequent tactical repositioning during multi-enemy kiting.
        if (!combatAnchor_ || distanceXY(start, *combatAnchor_) > 10.0) {
            combatAnchor_ = start;
            anchorSetAt_ = now;
        }
        const bool needReposition = (now - anchorSetAt_ >= 1.65) ||
            (now - threats_.last_damage_time() <= 0.35);

        const auto decision = choose(obs, start, zone, motion, muzzleHeight, threat, enemies, enemyInfos,
                                     projDodge, soundAlert, currentRoof, onRoof, close80, exposedNow, needReposition,
                                     tickStarted, hardDeadline);
        const auto& command = decision.command;

        Json action = {
            {"move", sanitizeMove(command.move)},
            {"boost", command.boost},
            {"jump", command.jump},
        };

        const auto& weapons = me.contains("weapons") && me["weapons"].is_array() ? me["weapons"] : Json::array();
        const bool laserCharging = isLaserCharging(weapons);
        const bool holdFireForRegen = regenMode_ && exposedNow == 0 && !laserCharging;
        const bool effectiveFireAllowed = decision.fireAllowed && !holdFireForRegen;

        const bool canJoin = duel && !underThreat && !regenMode_ && !peek_ && !hop_ && inZone && !window.urgent;
        const bool joining = effectiveFireAllowed && canJoin && opportunity_fire(obs, duel, arena_);

        const double myCubes = me.value("cubes", 0.0);
        const bool needsSecondary = countHeldWeapons(weapons) < 2;
        const bool allowCrates = !underThreat && !regenMode_ && inZone && !peek_ && !hop_ &&
            (myCubes < cubeStatCap_ || needsSecondary);
        const bool allowNpcs = !underThreat && !regenMode_ && inZone && !peek_ && !hop_ &&
            !window.urgent && myCubes < cubeStatCap_;
        const int close35 = static_cast<int>(std::count_if(enemyInfos.begin(), enemyInfos.end(), [&](const EnemyInfo& e) {
            return distanceXY(start, e.pos) <= 35.0;
        }));
        const bool allowLaserCharge = !window.urgent && inZone && !(threat && threat->laser_charging) &&
            (onRoof || close35 == 0) && !regenMode_;

        const auto shotDeadline = std::min(tickStarted + 34ms, Clock::now() + 5ms);
        std::optional<GunShot> shot;
        if ((effectiveFireAllowed || laserCharging) && !joining) {
            shot = threats_.visible_shot(obs, arena_, threat, shotDeadline, allowCrates, allowNpcs,
                                         allowLaserCharge, lastTargetId_);
        }
        if (shot && !shot->crate && !shot->npc && shot->target_id >= 0) {
            lastTargetId_ = shot->target_id;
        }
        const bool defending = effectiveFireAllowed && !joining && !(shot && shot->firing()) && !laserCharging &&
            return_fire(obs, threat, command.move, arena_, decision.returningToCover);

        // Aim priority: active shot > duel > visible front-armor facing (NEVER sell 1.5x rear armor!) > rear-hit snap > blind-spot sound > threat > scan.
        std::optional<Vec2> aim;
        if (shot) {
            aim = shot->aim;
        } else if (canJoin) {
            aim = retreat_aim(obs, duel);
        } else if (const auto armorYaw = frontArmorYaw(start, enemyInfos, motion.height, arena_)) {
            aim = Vec2{*armorYaw, 0.0};
        } else if (const auto rearYaw = threats_.recent_rear_hit_bearing(now)) {
            aim = Vec2{*rearYaw, 0.0};
        } else if (soundAlert && (soundAlert->laser_charge || (onRoof && soundAlert->near_jet) || soundAlert->near_hostile)) {
            aim = Vec2{soundAlert->bearing, (onRoof && soundAlert->near_jet) ? -0.35 : 0.0};
        } else if (threat && threat->position) {
            aim = retreat_aim(obs, threat);
        }
        if (aim) {
            action["aim"] = sanitizeAim(*aim);
        } else {
            action["aim"] = sanitizeAim(scanAim(me, command.move, zone.now));
        }

        std::array fire{false, false};
        if (joining || defending) {
            for (std::size_t i = 0; i < std::min(fire.size(), weapons.size()); ++i) {
                if (weapons[i].is_object() && weapons[i].value("kind", "") == std::string_view("gun")) {
                    fire[i] = true;
                    if (peek_) peek_->fired = true;
                    break;
                }
            }
        } else if (shot) {
            fire = shot->fire;
            if (peek_ && !shot->crate && !shot->npc && shot->firing()) {
                peek_->fired = true;
            }
        }
        action["fire"] = fire;

        const bool anyFire = fire[0] || fire[1] || laserCharging;
        // Hidden Auto-Reload: whenever safely tucked behind hard cover / roof core (exposedNow == 0)
        // or actively tucking on a roof in regenMode_, reload anytime mag < 8 (out of 12)!
        const double earlyReloadThreshold = (close70 == 0 && !underThreat) ? 8.0
            : ((exposedNow == 0 && (shelteredAt_.has_value() || onRoof)) || (onRoof && regenMode_)) ? 7.0 : 0.0;
        bool shouldReload = false;
        if (!anyFire) {
            shouldReload = std::any_of(weapons.begin(), weapons.end(), [earlyReloadThreshold](const Json& weapon) {
                if (!weapon.is_object() || weapon.value("reload", 0.0) > 0.0) {
                    return false;
                }
                if (weapon.value("kind", "") == std::string_view("gun")) {
                    return weapon.value("ammo", 0.0) <= earlyReloadThreshold;
                }
                return false;
            });
        }
        action["reload"] = shouldReload;

        // Explicit weapon pickup / upgrade swap (protocol.md §7: must send "take": id within 4 m).
        attachWeaponTake(obs, me, weapons, start, laserCharging, exposedNow, close35, action);

        if (const auto offerIt = obs.find("offer"); offerIt != obs.end() && !offerIt->is_null()) {
            action["pick"] = chooseOfferPick(*offerIt);
        }
        if (config_.value("debug", Json::object()).value("max_text_chars", 0) >= 16) {
            action["debug"] = {{"label", joining ? "duel-fire" : canJoin ? "duel-aim" : decision.label}};
        }
        return action;
    }

private:
    using Goals = std::vector<Vec3>;

    static std::optional<int> selfIdOf(const Json& obs) {
        const auto it = obs.find("self");
        if (it == obs.end() || !it->is_object()) return std::nullopt;
        const auto idIt = it->find("id");
        if (idIt == it->end() || !idIt->is_number()) return std::nullopt;
        const double v = idIt->get<double>();
        if (v >= 0 && v <= 1000000 && std::trunc(v) == v) return static_cast<int>(v);
        return std::nullopt;
    }

    static bool hasWeaponKind(const Json& weapons, std::string_view kind) {
        if (!weapons.is_array()) return false;
        return std::any_of(weapons.begin(), weapons.end(), [kind](const Json& w) {
            return w.is_object() && w.value("kind", "") == kind;
        });
    }

    static int countHeldWeapons(const Json& weapons) {
        if (!weapons.is_array()) return 0;
        int count = 0;
        for (const auto& w : weapons) {
            if (w.is_object()) ++count;
        }
        return count;
    }

    static bool isLaserCharging(const Json& weapons) {
        if (!weapons.is_array()) return false;
        return std::any_of(weapons.begin(), weapons.end(), [](const Json& w) {
            return w.is_object() && w.value("kind", "") == std::string_view("laser") &&
                w.value("charge", 0.0) > 0.0;
        });
    }

    static bool isGunReloading(const Json& weapons) {
        if (!weapons.is_array()) return false;
        return std::any_of(weapons.begin(), weapons.end(), [](const Json& w) {
            return w.is_object() && w.value("kind", "") == std::string_view("gun") &&
                (w.value("reload", 0.0) > 0.15 || w.value("ammo", 1.0) <= 0.0);
        });
    }

    static bool hasReadySecondary(const Json& weapons, double energy) {
        if (!weapons.is_array()) return false;
        for (const auto& w : weapons) {
            if (!w.is_object()) continue;
            const auto kind = w.value("kind", std::string{});
            if (kind == "gun") continue;
            if (w.value("cooldown", 0.0) <= 0.15) {
                if (kind != "lightning" || energy >= 6.0) {
                    return true;
                }
            }
        }
        return false;
    }

    void attachWeaponTake(const Json& obs, const Json& me, const Json& weapons,
                          const Vec3& start, bool laserCharging, int exposedNow, int close35, Json& action) const {
        (void)me;
        const auto it = obs.find("pickups");
        if (it == obs.end() || !it->is_array() || it->empty()) {
            return;
        }
        int emptySlot = -1;
        if (weapons.is_array()) {
            for (std::size_t i = 0; i < 2; ++i) {
                if (i >= weapons.size() || weapons[i].is_null()) {
                    emptySlot = static_cast<int>(i);
                    break;
                }
            }
        }
        int slot1Prio = 0;
        bool slot1Ready = false;
        if (weapons.is_array() && weapons.size() >= 2 && weapons[1].is_object()) {
            slot1Prio = weaponPriority(weapons[1].value("kind", std::string{}));
            slot1Ready = weapons[1].value("cooldown", 0.0) <= 0.15;
        }
        // Avoid swapping away a ready secondary mid-brawl (swapping incurs a 1.0-1.65s equip cooldown).
        const bool safeToSwap = !laserCharging && (exposedNow == 0 || !slot1Ready || close35 == 0);

        int bestId = -1;
        int bestPrio = 0;
        bool needDrop1 = false;
        for (const auto& p : *it) {
            if (!p.is_object()) continue;
            const auto pos = safeVec3(p, "pos", {infinity, infinity, 0.0});
            if (!std::isfinite(pos[0])) continue;
            if (distanceXY(start, pos) > lootPickupRadius_ - 0.05 || std::abs(start[2] - pos[2]) > 3.9) {
                continue;
            }
            const auto kind = p.value("kind", std::string{});
            if (kind.empty() || kind == "gun" || hasWeaponKind(weapons, kind)) {
                continue;
            }
            const int prio = weaponPriority(kind);
            if (prio <= bestPrio) continue;
            const auto idIt = p.find("id");
            if (idIt == p.end() || !idIt->is_number()) continue;
            const int pid = static_cast<int>(idIt->get<double>());
            if (emptySlot >= 0) {
                bestId = pid;
                bestPrio = prio;
                needDrop1 = false;
            } else if (safeToSwap && prio > slot1Prio) {
                bestId = pid;
                bestPrio = prio;
                needDrop1 = true;
            }
        }
        if (bestId >= 0) {
            action["take"] = bestId;
            if (needDrop1) {
                action["drop"] = 1;
            }
        }
    }

    std::vector<Vec3> localEnemies(const Json& obs, const Vec3& start,
                                   const std::optional<Threat>& threat, double ownHeight) const {
        std::vector<Vec3> enemies;
        const auto selfId = selfIdOf(obs);
        const auto mechsIt = obs.find("mechs");
        if (mechsIt != obs.end() && mechsIt->is_array()) {
            for (const auto& actor : *mechsIt) {
                if (!actor.is_object() || actor.value("kind", "") != std::string_view("mech")) continue;
                if (selfId && actor.contains("id") && actor["id"].is_number() &&
                    static_cast<int>(actor["id"].get<double>()) == *selfId) {
                    continue;
                }
                const auto point = safeVec3(actor, "pos", {infinity, infinity, 0.0});
                if (!std::isfinite(point[0])) continue;
                const double dist = distanceXY(start, point);
                if (dist <= 35.0 || (dist <= 95.0 && wallCover(start, point, ownHeight, arena_) < 3)) {
                    enemies.push_back(point);
                }
            }
        }
        if (threat && threat->position && threat->hostile &&
            std::none_of(enemies.begin(), enemies.end(), [&](const Vec3& enemy) {
                return distanceXY(*threat->position, enemy) < 1.0;
            })) {
            enemies.push_back(*threat->position);
        }
        return enemies;
    }

    Vec2 scanAim(const Json& me, const Vec2& move, double now) {
        if (const auto soundYaw = threats_.sound_heading(now)) {
            scanHeading_ = *soundYaw;
            return {*soundYaw, 0.0};
        }
        if (std::hypot(move[0], move[1]) > 0.12) {
            scanHeading_ = std::atan2(move[1], move[0]);
        }
        if (!scanHeading_) {
            scanHeading_ = safeVec2(me, "aim")[0];
        }
        const double cycle = std::fmod(now, 8.0);
        const double offset = cycle >= 6.6 ? std::numbers::pi : 1.48 * std::sin(cycle * (2 * std::numbers::pi / 3.3));
        return {std::remainder(*scanHeading_ + offset, 2 * std::numbers::pi), 0.0};
    }

    Goals rotationGoals(const Vec3& start, const Zone& zone, const Motion& motion) const {
        if (zone.inNext(start, motion.radius + 5)) {
            return {start};
        }
        const auto [x, y] = zone.nextCenter;
        const auto [minX, minY, maxX, maxY] = arena_.bounds;
        const double margin = motion.radius + 0.3;
        const double safe = std::max(0.0, zone.nextRadius - zone.buffer(zone.nextRadius, motion.radius + 6));
        const double angle = std::atan2(start[1] - y, start[0] - x);
        constexpr std::array offsets{0, 24, -24, 50, -50, 85, -85, 135, -135, 180};

        auto collect = [&](bool allowElevated) {
            Goals points;
            // Sample primarily along the outer safe rim to avoid getting 360-deg pinched in the center.
            for (const double radius : {safe * 0.94, safe * 0.80, safe * 0.58, 0.0}) {
                for (const int offset : offsets) {
                    if (radius == 0.0 && offset != 0) {
                        break;
                    }
                    const double yaw = angle + radians(offset);
                    const Vec2 point{
                        std::clamp(x + radius * std::cos(yaw), minX + margin, maxX - margin),
                        std::clamp(y + radius * std::sin(yaw), minY + margin, maxY - margin),
                    };
                    if (!zone.inNext(point) || std::any_of(points.begin(), points.end(), [&](const Vec3& p) {
                            return distanceXY(point, p) <= 3;
                        })) {
                        continue;
                    }
                    const double ceiling = allowElevated ? infinity : arena_.ground(point[0], point[1]) + motion.step + 0.05;
                    if (const auto goal = navigator_.support(point, motion, ceiling)) {
                        points.push_back(*goal);
                    }
                }
            }
            return points;
        };

        Goals points = collect(false);
        if (points.empty()) {
            points = collect(true);
        }
        // Prefer building-adjacent cover positions along the outer rim over exposed open-lawn center spots.
        auto goalCost = [&](const Vec3& p) {
            const double travel = distanceXY(start, p);
            const double wallDist = arena_.distanceToNearestWall(p[0], p[1], p[2], 26.0);
            const double coverBonus = wallDist <= 16.0 ? -36.0 : (wallDist <= 24.0 ? -16.0 : 0.0);
            const double distFromCenter = distanceXY(p, zone.nextCenter);
            const double centerPenalty = (zone.nextRadius >= 60.0 && distFromCenter < safe * 0.45) ? 28.0 : 0.0;
            return travel + coverBonus + centerPenalty;
        };
        std::stable_sort(points.begin(), points.end(), [&](const Vec3& a, const Vec3& b) {
            return goalCost(a) < goalCost(b);
        });
        if (points.size() > 16) {
            points.resize(16);
        }
        return points;
    }

    static bool hasReadyMortar(const Json& weapons) {
        if (!weapons.is_array()) return false;
        return std::any_of(weapons.begin(), weapons.end(), [](const Json& w) {
            return w.is_object() && w.value("kind", "") == std::string_view("mortar") &&
                w.value("cooldown", 0.0) <= 0.15;
        });
    }

    Decision choose(const Json& obs, const Vec3& start, const Zone& zone, const Motion& motion,
                    double muzzleHeight, const std::optional<Threat>& threat,
                    const std::vector<Vec3>& enemies, const std::vector<EnemyInfo>& enemyInfos,
                    const std::optional<ProjectileDodge>& projDodge,
                    const std::optional<SoundAlert>& soundAlert,
                    const RooftopNode* currentRoof, bool onRoof, int close80, int exposedNow, bool needReposition,
                    Clock::time_point tickStarted, Clock::time_point hardDeadline) {
        const auto& me = obs.contains("self") && obs["self"].is_object() ? obs["self"] : Json::object();
        const auto phase = std::pair{zone.start, zone.end};
        if (rotationPhase_ != phase) {
            rotationPhase_ = phase;
            goalDeadline_ = -infinity;
            routeEstimate_.reset();
            routeEstimatePos_.reset();
            goals_.clear();
            navigator_.reset();
        }
        if (routeEstimate_ && routeEstimatePos_ && distanceXY(start, *routeEstimatePos_) > 18.0) {
            routeEstimate_.reset();
            routeEstimatePos_.reset();
        }
        const auto window = rotationWindow(start, zone, motion, routeEstimate_);
        const bool inZone = me.value("in_zone", false);
        const bool grounded = me.value("grounded", true);
        const double hp = me.value("hp", 1400.0);
        const double maxHp = std::max(1.0, me.value("max_hp", 1400.0));
        const double hpFrac = hp / maxHp;
        const double energy = me.value("energy", 0.0);
        const double maxEnergy = std::max(1.0, me.value("max_energy", 100.0));
        const auto& weapons = me.contains("weapons") && me["weapons"].is_array() ? me["weapons"] : Json::array();

        // Priority 1: Urgent mortar shell dodge BEFORE ground hops or walking to a takeoff point!
        if (projDodge && projDodge->urgent_mortar && grounded && inZone) {
            if (flight_ && !flight_->airborne) flight_.reset();
            const double baseYaw = std::atan2(projDodge->direction[1], projDodge->direction[0]);
            const bool canSprint = energy > 0.2 * maxEnergy;
            for (const int off : {0, 35, -35, 70, -70, 110, -110}) {
                const double yaw = baseYaw + radians(off);
                const Vec2 move{std::cos(yaw), std::sin(yaw)};
                if (canSprint && navigator_.guard(move, true, me, zone, motion)) {
                    return {Command{move, true}, "dodge-mortar", true};
                }
                if (navigator_.guard(move, false, me, zone, motion)) {
                    return {Command{move, false}, "dodge-mortar", true};
                }
            }
        }

        // Priority 2: Active airborne hop or laser/wall/true-cover defensive hop.
        if (const auto evasive = evasiveMove(me, start, zone, motion, threat, enemies, window,
                                             std::min(hardDeadline, Clock::now() + 5ms))) {
            if (flight_ && !flight_->airborne) flight_.reset();
            return *evasive;
        }

        // Priority 3: True blind-spot heard laser charge ONLY (never interrupt urgent zone rotation or visible fights!).
        if (soundAlert && soundAlert->laser_charge && enemyInfos.empty() && !window.urgent &&
            !(threat && threat->laser_charging) && grounded && inZone && (!flight_ || !flight_->airborne)) {
            const double perpBase = soundAlert->bearing + strafeSign_ * (std::numbers::pi * 0.5);
            const Vec3 virtualSrc{start[0] + 60.0 * std::cos(soundAlert->bearing),
                                  start[1] + 60.0 * std::sin(soundAlert->bearing), start[2]};
            std::optional<Decision> bestDodge;
            double bestScore = -infinity;
            for (const int off : {0, 180, 25, -25, 155, -155}) {
                const double yaw = perpBase + radians(off);
                const Vec2 move{std::cos(yaw), std::sin(yaw)};
                const bool sprint = energy > 0.35 * maxEnergy && navigator_.guard(move, true, me, zone, motion);
                if (!sprint && !navigator_.guard(move, false, me, zone, motion)) continue;
                const Vec3 probe{start[0] + move[0] * 6.0, start[1] + move[1] * 6.0, start[2]};
                const double score = 5.0 * wallCover(probe, virtualSrc, motion.height, arena_);
                if (score > bestScore) {
                    bestScore = score;
                    bestDodge = Decision{{move, sprint}, "sound-laser", true};
                }
            }
            if (bestDodge) {
                if (flight_ && !flight_->airborne) flight_.reset();
                return *bestDodge;
            }
        }

        if (flight_ && !flight_->airborne && threat && threat->laser_charging) flight_.reset();
        if (const auto command = fly(me, start, zone, motion)) {
            return {*command, "rotate-air"};
        }

        // Tactical rooftop seizure: allow nearby campus roofs (takeoff <= 18m), avoid mid-air beaming at < 35% HP,
        // and never abandon safe hard cover while actively regenerating!
        const bool insideNext = zone.inNext(start, motion.radius + 5);
        const bool underThreat = threat && (threat->hostile || threat->damaged_recently || threat->laser_charging);
        const bool safeToClimb = (hpFrac >= 0.35 || exposedNow == 0) && !(regenMode_ && exposedNow == 0);
        const bool wantRoof = !onRoof && grounded && inZone && !window.urgent && !isLaserCharging(weapons) &&
            safeToClimb && energy >= 0.55 * maxEnergy && zone.now >= considerRoofAt_ &&
            (close80 >= 2 || (regenMode_ && exposedNow > 0) ||
             (insideNext && !underThreat && !regenMode_ && countHeldWeapons(weapons) >= 2 && me.value("cubes", 0.0) >= 4.0));
        if (wantRoof) {
            considerRoofAt_ = zone.now + 1.5;
            const auto roofs = arena_.nearbyRooftops(start, 38.0, 5.5, 26.0, motion.radius, motion.height);
            const auto roofDeadline = std::min(hardDeadline, Clock::now() + 4ms);
            for (const auto& roof : roofs) {
                if (Clock::now() >= roofDeadline) break;
                if (const auto plan = navigator_.flightToRoof(start, roof, zone, motion, energy, maxEnergy, roofDeadline)) {
                    if (distanceXY(start, plan->takeoff) <= 18.0) {
                        flight_ = plan;
                        if (const auto cmd = fly(me, start, zone, motion)) {
                            return {*cmd, "climb-roof", true};
                        }
                    }
                }
            }
        }

        // Rooftop-to-Rooftop jump link when on a roof and either rotating or pinched by 2+ enemies!
        if (currentRoof && grounded && !isLaserCharging(weapons) && energy >= 0.50 * maxEnergy &&
            zone.now >= considerRoofAt_ && !currentRoof->jumpLinks.empty()) {
            const bool wantHop = (!insideNext && window.urgent) || (close80 >= 2 && exposedNow >= 2 && hpFrac < 0.55);
            if (wantHop) {
                considerRoofAt_ = zone.now + 1.2;
                const auto hopDeadline = std::min(hardDeadline, Clock::now() + 3ms);
                for (const auto& link : currentRoof->jumpLinks) {
                    if (Clock::now() >= hopDeadline) break;
                    if (!insideNext && distanceXY(link.landing, zone.nextCenter) >= distanceXY(start, zone.nextCenter) - 4.0) {
                        continue;
                    }
                    if (const auto plan = navigator_.flightToRoof(start, link.landing, zone, motion, energy, maxEnergy, hopDeadline)) {
                        if (distanceXY(start, plan->takeoff) <= 16.0) {
                            flight_ = plan;
                            if (const auto cmd = fly(me, start, zone, motion)) {
                                return {*cmd, "roof-hop", true};
                            }
                        }
                    }
                }
            }
        }

        // Capture a landed or safe nearby supply drop (never sprint 95m at low HP into the enemy squad!).
        if (inZone && !window.urgent && !(threat && threat->laser_charging)) {
            if (const auto cap = supplyDropCapture(obs, start, zone, motion, enemyInfos, underThreat)) {
                return *cap;
            }
        }

        // User's Storm-Finish Rule: allow brief early/mid storm tanking (dps <= 60, ample HP, near border)
        // to finish a low-HP 1v1 kill before rotating into the safe zone!
        const bool canFinishInStorm = (!inZone || window.urgent) && threat && threat->position &&
            !threat->is_npc && close80 <= 1 && threat->hp <= 420.0 &&
            distanceXY(start, *threat->position) <= 62.0 &&
            zone.dps <= 60.0 && hp >= 580.0 && hpFrac >= 0.42 &&
            zone.clearance(start, zone.now) >= -36.0;

        if (threat && threat->position && ((!window.urgent && inZone) || canFinishInStorm)) {
            const double threatDist = distanceXY(start, *threat->position);
            const int alive = obs.value("alive", 16);
            // Early snowball hunt: actively engage & finish isolated 1v1 weak bots up to 140m instead of running away!
            const bool earlyHunt1v1 = alive >= 11 && enemyInfos.size() == 1 && exposedNow == 1 &&
                hpFrac >= 0.65 && !regenMode_ && !threat->is_npc && threatDist <= 140.0;
            const bool activeCombat = threat->hostile || threat->damaged_recently ||
                threat->laser_charging || threatDist <= 70.0 || regenMode_ || earlyHunt1v1 || canFinishInStorm;

            if (activeCombat) {
                if (zone.now >= nextStrafeFlipAt_ || (zone.now - threats_.last_damage_time() <= 0.15 && zone.now >= nextStrafeFlipAt_ - 0.8)) {
                    strafeSign_ = -strafeSign_;
                    nextStrafeFlipAt_ = zone.now + 1.8;
                }
                const bool reloadingNoBackup = isGunReloading(weapons) && !hasReadySecondary(weapons, energy);

                // Don't peek out into fire if we can safely lob a ready mortar over the wall!
                if (!onRoof && !regenMode_ && !reloadingNoBackup && !hasReadyMortar(weapons)) {
                    std::vector<Vec3> allPos;
                    allPos.reserve(enemyInfos.size());
                    for (const auto& e : enemyInfos) allPos.push_back(e.pos);
                    if (const auto peeking = peekCommand(obs, start, zone, motion, muzzleHeight, *threat, allPos, window)) {
                        return *peeking;
                    }
                }

                CombatContext ctx;
                ctx.hp = me.value("hp", 1400.0);
                ctx.maxHp = std::max(1.0, me.value("max_hp", 1400.0));
                ctx.energy = energy;
                ctx.maxEnergy = maxEnergy;
                ctx.cubes = me.value("cubes", 0.0);
                ctx.muzzleHeight = muzzleHeight;
                ctx.strafeSign = strafeSign_;
                ctx.hasLightning = hasWeaponKind(weapons, "lightning");
                ctx.hasLaser = hasWeaponKind(weapons, "laser");
                ctx.hasMortar = hasWeaponKind(weapons, "mortar");
                ctx.hasReadyMortar = hasReadyMortar(weapons);
                ctx.laserCharging = isLaserCharging(weapons);
                ctx.reloading = reloadingNoBackup;
                ctx.wantRegen = regenMode_;
                ctx.needReposition = needReposition;
                ctx.onRoof = onRoof;
                ctx.currentRoof = currentRoof;
                ctx.anchor = combatAnchor_.value_or(start);
                const auto vel3 = safeVec3(me, "vel");
                ctx.velocity = {vel3[0], vel3[1]};
                if (projDodge && !projDodge->urgent_mortar) {
                    ctx.bulletDodge = projDodge->direction;
                }
                // When caught in open ground (e.g., OSU Oval lawn), pull the fight toward the nearest building or tree cover!
                if (!onRoof && arena_.distanceToNearestWall(start[0], start[1], start[2], 24.0) > 16.0) {
                    if (const auto anchor = arena_.nearestBuildingAnchor(start, 140.0, motion.radius, zone.nextCenter)) {
                        if (zone.clearance(*anchor, zone.now + 10.0) >= motion.radius) {
                            ctx.coverPull = *anchor;
                        }
                    }
                    if (threatDist > 48.0) {
                        if (const auto treeSpot = arena_.nearestTreeCover(start, *threat->position, 32.0, motion.radius)) {
                            if (zone.clearance(*treeSpot, zone.now + 8.0) >= motion.radius) {
                                ctx.treeCover = *treeSpot;
                            }
                        }
                    }
                }

                auto tacticalEnemies = enemyInfos;
                if (threat->is_npc || std::none_of(tacticalEnemies.begin(), tacticalEnemies.end(), [&](const EnemyInfo& e) {
                        return distanceXY(e.pos, *threat->position) < 1.0;
                    })) {
                    EnemyInfo extra;
                    extra.id = threat->actor_id.value_or(-2);
                    extra.pos = *threat->position;
                    extra.hp = threat->hp;
                    extra.radius = threat->attack_radius;
                    extra.height = threat->attack_height;
                    extra.is_npc = threat->is_npc;
                    extra.has_lightning = threat->has_lightning;
                    extra.has_laser = threat->has_laser;
                    extra.has_mortar = threat->has_mortar;
                    extra.charging = threat->laser_charging;
                    extra.facing_us = threat->facing_us;
                    tacticalEnemies.push_back(extra);
                }

                const auto combatMove = planCombatPosition(start, *threat, tacticalEnemies, ctx, zone, motion, arena_,
                                                           navigator_, std::min(hardDeadline, Clock::now() + 7ms));
                if (combatMove) {
                    Zone guardZone = zone;
                    if (canFinishInStorm) {
                        guardZone.radius += 45.0;
                        guardZone.nextRadius += 45.0;
                    }
                    if (combatMove->hardCover && distanceXY(start, combatMove->target) < 0.9) {
                        if (!shelteredAt_) shelteredAt_ = zone.now;
                        return {groundCommand(start, std::nullopt, me, guardZone, motion, false, enemies), combatMove->label, true};
                    }
                    shelteredAt_.reset();
                    const bool sprint = grounded && combatMove->boost;
                    const auto cmd = groundCommand(start, combatMove->target, me, guardZone, motion, sprint, {});
                    if (std::hypot(cmd.move[0], cmd.move[1]) > 0.12) {
                        return {cmd, combatMove->label, true};
                    }
                    strafeSign_ = -strafeSign_;
                }

                std::vector<Vec3> allPos;
                allPos.reserve(enemyInfos.size());
                for (const auto& e : enemyInfos) allPos.push_back(e.pos);
                auto retreat = retreatTarget(start, *threat, allPos, zone, motion,
                                             std::min(hardDeadline, Clock::now() + 4ms));
                if (retreat) {
                    if (retreat->hardCover && distanceXY(start, retreat->target) < 0.9) {
                        if (!shelteredAt_) shelteredAt_ = zone.now;
                        return {groundCommand(start, std::nullopt, me, zone, motion, false, enemies), "hold-wall", true};
                    }
                    shelteredAt_.reset();
                    const bool boost = grounded && threat->hostile && threatDist < 55.0 &&
                        energy > 0.35 * maxEnergy;
                    const auto command = groundCommand(start, retreat->target, me, zone, motion, boost, enemies);
                    if (std::hypot(command.move[0], command.move[1]) > 0.12) {
                        return {command, "withdraw", true};
                    }
                    retreat_.reset();
                }
            } else {
                peek_.reset();
                retreat_.reset();
            }
        } else {
            peek_.reset();
            retreat_.reset();
        }
        shelteredAt_.reset();
        return rotate(obs, start, zone, motion, threat, enemies, enemyInfos, onRoof, window, tickStarted, hardDeadline);
    }

    std::optional<Decision> supplyDropCapture(const Json& obs, const Vec3& start, const Zone& zone,
                                              const Motion& motion, const std::vector<EnemyInfo>& enemyInfos,
                                              bool underThreat) {
        const auto it = obs.find("drops");
        if (it == obs.end() || !it->is_array() || it->empty()) {
            return std::nullopt;
        }
        const auto selfId = selfIdOf(obs);
        const int closeEnemies = static_cast<int>(std::count_if(enemyInfos.begin(), enemyInfos.end(), [&](const EnemyInfo& e) {
            return distanceXY(start, e.pos) <= 55.0;
        }));
        if (closeEnemies >= 2) {
            return std::nullopt;
        }
        const auto& me = obs.contains("self") && obs["self"].is_object() ? obs["self"] : Json::object();
        for (const auto& drop : *it) {
            if (!drop.is_object()) continue;
            const auto pos = safeVec3(drop, "pos", {infinity, infinity, 0.0});
            if (!std::isfinite(pos[0])) continue;
            const double landsIn = drop.value("lands_in", 99.0);
            const double radius = drop.value("radius", 8.0);
            const double dist = distanceXY(start, pos);
            if (landsIn > 10.0 || dist > 42.0) {
                continue;
            }
            if (zone.clearance(horizontal(pos), zone.now + landsIn + 4.5) < motion.radius + 1.0) {
                continue;
            }
            if (drop.contains("holder") && drop["holder"].is_number() && selfId &&
                static_cast<int>(drop["holder"].get<double>()) != *selfId) {
                continue;
            }
            if (dist <= radius - 1.5 && landsIn <= 0.15) {
                return Decision{groundCommand(start, std::nullopt, me, zone, motion, false), "cap-drop", true};
            }
            if (underThreat || closeEnemies > 0) {
                continue;
            }
            const auto target = navigator_.support(horizontal(pos), motion, pos[2] + 3.0);
            if (target && navigator_.direct(start, *target, zone, motion)) {
                const bool sprint = me.value("grounded", true) &&
                    me.value("energy", 0.0) > 0.45 * me.value("max_energy", 100.0);
                return Decision{groundCommand(start, *target, me, zone, motion, sprint), "seek-drop", true};
            }
        }
        return std::nullopt;
    }

    bool wallBehind(const Vec3& start, const Vec2& away, const Motion& motion) const {
        const double reach = motion.radius + 2.2;
        for (const double height : {0.55, motion.height * 0.45}) {
            const Vec3 from{start[0], start[1], start[2] + height};
            const Vec3 end{from[0] + reach * away[0], from[1] + reach * away[1], from[2]};
            if (arena_.blocksPrismLine(from, end)) return true;
        }
        return false;
    }

    bool keepsRotation(const Vec3& from, const Vec3& to, const Zone& zone, const RotationWindow& window) const {
        return !window.urgent || zone.inNext(from) ||
            distanceXY(to, zone.nextCenter) <= distanceXY(from, zone.nextCenter) + 0.05;
    }

    std::optional<Decision> evasiveMove(const Json& me, const Vec3& start, const Zone& zone, const Motion& motion,
                                       const std::optional<Threat>& threat, const std::vector<Vec3>& enemies,
                                       const RotationWindow& window, Clock::time_point deadline) {
        const bool grounded = me.value("grounded", false);
        if (hop_) {
            if (!grounded && zone.now - hop_->started < hop_->path.duration + 0.35) {
                const auto move = zone.now - hop_->started + 1e-6 < hop_->path.steerAfter ? Vec2{} : hop_->path.launch;
                if (navigator_.guard(move, false, me, zone, motion)) {
                    return Decision{{move}, hop_->label};
                }
                const auto command = groundCommand(start, hop_->path.landing, me, zone, motion, false, enemies);
                return Decision{command, hop_->label};
            }
            hop_.reset();
        }
        if (!grounded || !me.value("in_zone", false) || !threat || !threat->position) return {};
        const Vec3& enemy = *threat->position;
        const auto away = direction(enemy, start);
        if (std::hypot(away[0], away[1]) < 0.1) return {};
        const bool laser = threat->laser_charging;
        const bool exposed = wallCover(start, enemy, motion.height, arena_) < 3;
        const bool blocked = threat->facing_us && wallBehind(start, away, motion);
        if (zone.now >= nextHopAt_ && exposed && (laser || threat->damaged_recently || blocked)) {
            const auto path = planDefensiveHop(me, *threat, enemies, zone, motion, arena_, navigator_, laser, blocked, deadline);
            // Allow defensive hop when it gains cover (>=2/3), or under close-range/multi-enemy pressure;
            // only skip pointless open-field single-bullet hops at long range (> 45m, 1v1, 0 cover).
            const bool goodHop = path && (wallCover(path->landing, enemy, motion.height, arena_) >= 2 ||
                                          distanceXY(start, enemy) <= 45.0 || enemies.size() >= 2);
            if (path && (laser || blocked || goodHop) && keepsRotation(start, path->landing, zone, window)) {
                const auto label = laser ? "laser-hop" : blocked ? "wall-hop" : "cover-hop";
                hop_ = ActiveHop{*path, zone.now, label};
                nextHopAt_ = zone.now + 2.4;
                retreat_.reset();
                shelteredAt_.reset();
                peek_.reset();
                return Decision{{path->steerAfter > 0 ? Vec2{} : path->launch, false, true}, label};
            }
        }
        if (!laser) return {};

        const auto velocity = safeVec3(me, "vel");
        std::optional<Decision> best;
        double bestScore = -infinity;
        for (const double side : {-1.0, 1.0}) {
            const Vec2 point{start[0] + 1.4 * away[0] - side * 6.0 * away[1],
                             start[1] + 1.4 * away[1] + side * 6.0 * away[0]};
            const auto target = navigator_.support(point, motion, start[2] + motion.step);
            if (!target || !keepsRotation(start, *target, zone, window) ||
                !navigator_.direct(start, *target, zone, motion)) continue;
            const auto move = direction(start, point);
            const bool sprint = me.value("energy", 0.0) > 0.35 * me.value("max_energy", 100.0) &&
                navigator_.guard(move, true, me, zone, motion);
            if (!sprint && !navigator_.guard(move, false, me, zone, motion)) continue;
            const double score = 4 * wallCover(*target, enemy, motion.height, arena_) +
                0.16 * (velocity[0] * move[0] + velocity[1] * move[1]) -
                0.15 * approachPenalty(start, point, enemies);
            if (score > bestScore) {
                bestScore = score;
                best = Decision{{move, sprint}, "laser-strafe"};
            }
        }
        if (best) {
            retreat_.reset();
            shelteredAt_.reset();
            peek_.reset();
        }
        return best;
    }

    std::optional<RetreatPlan> retreatTarget(const Vec3& start, const Threat& threat, const std::vector<Vec3>& enemies,
                                            const Zone& zone, const Motion& motion, Clock::time_point deadline) {
        if (retreat_ && retreat_->opponent == threat.actor_id && zone.now < retreat_->expires &&
            distanceXY(start, retreat_->target) > 0.9 &&
            navigator_.direct(start, retreat_->target, zone, motion)) {
            return RetreatPlan{retreat_->target, false};
        }
        auto plan = planRetreat(start, threat, enemies, zone, motion, arena_, navigator_, deadline);
        retreat_.reset();
        if (plan && distanceXY(start, plan->target) > 0.9) {
            const double hold = std::clamp(distanceXY(start, plan->target) / motion.walk + 0.45, 0.65, 2.3);
            retreat_ = RetreatIntent{plan->target, threat.actor_id, zone.now + hold};
        }
        return plan;
    }

    std::optional<Decision> peekCommand(const Json& obs, const Vec3& start, const Zone& zone,
                                        const Motion& motion, double muzzleHeight, const Threat& threat,
                                        const std::vector<Vec3>& enemies, const RotationWindow& window) {
        const auto& me = obs.contains("self") && obs["self"].is_object() ? obs["self"] : Json::object();
        if (peek_ && (!threat.actor_id || peek_->opponent != *threat.actor_id ||
                      zone.now - peek_->started > 2.4 || !navigator_.direct(start, peek_->cover, zone, motion))) {
            peek_.reset();
        }
        if (!peek_) {
            const auto& weapons = me.contains("weapons") && me["weapons"].is_array() ? me["weapons"] : Json::array();
            const bool gunReady = std::any_of(weapons.begin(), weapons.end(), [](const Json& weapon) {
                return weapon.is_object() && weapon.value("kind", "") == std::string_view("gun") &&
                    weapon.value("ammo", 0.0) > 0 && weapon.value("reload", 0.0) <= 0 && weapon.value("cooldown", 0.0) <= 0.1;
            });
            const int closeEnemies = static_cast<int>(std::count_if(enemies.begin(), enemies.end(), [&](const Vec3& e) {
                return distanceXY(start, e) <= 95.0;
            }));
            if (!shelteredAt_ || zone.now - *shelteredAt_ < 0.35 || zone.now < nextPeekAt_ || !gunReady ||
                !me.value("grounded", false) || me.value("hp", 0.0) < 0.45 * me.value("max_hp", 1400.0) ||
                window.secondsUntilRisk < window.estimatedTravel + 8 || closeEnemies > 2 || !threat.actor_id ||
                !threat.position || distanceXY(start, *threat.position) < 15 ||
                wallCover(start, *threat.position, motion.height, arena_) != 3) {
                return {};
            }
            const Vec2 toward = direction(start, *threat.position);
            const Vec3 enemyTorso{(*threat.position)[0], (*threat.position)[1],
                                  (*threat.position)[2] + threat.attack_height * 0.5};
            for (const double side : {3.5, -3.5, 5.5, -5.5}) {
                const Vec2 xy{start[0] - toward[1] * side, start[1] + toward[0] * side};
                const auto edge = navigator_.support(xy, motion, start[2] + motion.step);
                if (!edge) continue;
                const Vec3 edgeMuzzle{(*edge)[0], (*edge)[1], (*edge)[2] + muzzleHeight};
                // Ensure peeking out does not expose us to a second enemy!
                const bool safeFromOthers = std::all_of(enemies.begin(), enemies.end(), [&](const Vec3& e) {
                    return distanceXY(e, *threat.position) < 0.5 || distanceXY(*edge, e) > 160.0 ||
                        wallCover(*edge, e, motion.height, arena_) == 3;
                });
                if (safeFromOthers && zone.clearance(*edge, zone.now + 2) > motion.radius + 1 &&
                    navigator_.direct(start, *edge, zone, motion) &&
                    navigator_.direct(*edge, start, zone, motion) &&
                    !arena_.blocksLine(edgeMuzzle, enemyTorso)) {
                    peek_ = Peek{start, *edge, *threat.actor_id, zone.now};
                    nextPeekAt_ = zone.now + 2.2;
                    shelteredAt_.reset();
                    break;
                }
            }
            if (!peek_) return {};
        }
        auto& peek = *peek_;
        peek.returning |= peek.fired || threat.laser_charging ||
            distanceXY(start, peek.edge) < 0.95 || zone.now - peek.started > 0.75;
        if (peek.returning && distanceXY(start, peek.cover) < 0.8) {
            peek_.reset();
            return {};
        }
        const auto target = peek.returning ? peek.cover : peek.edge;
        const auto move = direction(start, target, std::min(1.0, distanceXY(start, target) / 1.3));
        if (!navigator_.guard(move, false, me, zone, motion)) {
            peek_.reset();
            return {};
        }
        return Decision{{move}, peek.returning ? "peek-return" : "peek-edge", !peek.fired, peek.returning};
    }

    Decision rotate(const Json& obs, const Vec3& start, const Zone& zone, const Motion& motion,
                    const std::optional<Threat>& threat, const std::vector<Vec3>& enemies,
                    const std::vector<EnemyInfo>& enemyInfos, bool onRoof,
                    const RotationWindow& window, Clock::time_point tickStarted,
                    Clock::time_point hardDeadline) {
        const auto& me = obs.contains("self") && obs["self"].is_object() ? obs["self"] : Json::object();
        const bool insideNext = zone.inNext(start, motion.radius + 5);
        const double energy = me.value("energy", 0.0), maximum = std::max(1.0, me.value("max_energy", 100.0));
        const bool grounded = me.value("grounded", true);
        const bool outside = !me.value("in_zone", true);
        const auto& weapons = me.contains("weapons") && me["weapons"].is_array() ? me["weapons"] : Json::array();
        const bool hostileNow = threat && (threat->hostile || threat->damaged_recently || threat->laser_charging);
        const bool needsGear = me.value("cubes", 0.0) < 8.0 || countHeldWeapons(weapons) < 2;

        // Decide slowEdgePhase BEFORE running A* so we NEVER call navigator_.plan() with two competing roles!
        const bool slowEdgePhase = !outside && !window.urgent && !insideNext && !onRoof && !hostileNow &&
            needsGear && zone.radius >= 150.0 &&
            window.secondsUntilRisk > window.estimatedTravel + 14.0;

        Goals route;
        bool complete = insideNext;
        std::optional<Vec3> target;
        const char* labelOverride = nullptr;

        if (slowEdgePhase) {
            if (const auto outerLoot = outerRimLoot(obs, start, enemyInfos, zone, motion, weapons)) {
                target = *outerLoot;
                labelOverride = "edge-loot";
            } else if (arena_.distanceToNearestWall(start[0], start[1], start[2], 26.0) > 22.0) {
                if (const auto anchor = arena_.nearestBuildingAnchor(start, 110.0, motion.radius, zone.nextCenter)) {
                    if (zone.clearance(*anchor, zone.now + 12.0) > motion.radius + 2.0) {
                        if (const auto pt = navigator_.support(*anchor, motion, arena_.ground((*anchor)[0], (*anchor)[1]) + motion.step + 0.1)) {
                            auto bPlan = navigator_.plan(start, {*pt}, zone, motion, 8500,
                                                         std::min(hardDeadline, tickStarted + 22ms));
                            if (!bPlan.route.empty()) {
                                target = bPlan.route.front();
                                labelOverride = "edge-cover";
                            }
                        }
                    }
                }
            } else if (distanceXY(start, zone.nextCenter) > zone.nextRadius + 25.0) {
                const double toCenter = std::atan2(zone.nextCenter[1] - start[1], zone.nextCenter[0] - start[0]);
                const double rimAngle = toCenter + strafeSign_ * radians(65.0);
                const Vec2 rimXY{start[0] + 32.0 * std::cos(rimAngle), start[1] + 32.0 * std::sin(rimAngle)};
                if (zone.clearance(rimXY, zone.now + 10.0) > motion.radius + 3.0) {
                    if (const auto pt = navigator_.support(rimXY, motion, arena_.ground(rimXY[0], rimXY[1]) + motion.step + 0.1)) {
                        auto rPlan = navigator_.plan(start, {*pt}, zone, motion, 8600,
                                                     std::min(hardDeadline, tickStarted + 22ms));
                        if (!rPlan.route.empty()) {
                            target = rPlan.route.front();
                            labelOverride = "edge-patrol";
                        }
                    }
                }
            }
        }

        if (!target && !insideNext) {
            const bool reachedPrevious = goals_.size() == 1 && distanceXY(start, goals_.front()) < 0.85;
            if (zone.now >= goalDeadline_ || reachedPrevious) {
                auto desired = rotationGoals(start, zone, motion);
                if (goals_.empty() || desired.empty() || reachedPrevious || distanceXY(goals_.front(), desired.front()) > 10) {
                    goals_ = std::move(desired);
                    ++routeId_;
                }
                goalDeadline_ = zone.now + 1.1;
            }
            auto plan = navigator_.plan(start, goals_, zone, motion, routeId_,
                                        std::min(hardDeadline, tickStarted + 22ms));
            route = std::move(plan.route);
            complete = plan.complete;
        }
        const double eta = navigator_.pathTime(start, route, motion.walk);
        if (complete && !slowEdgePhase) {
            routeEstimate_ = eta;
            routeEstimatePos_ = start;
        }
        const double remaining = std::max(0.0, zone.end - zone.now);
        if (!target && grounded && zone.now >= considerFlightAt_ && !insideNext && energy > maximum * 0.35 &&
            (goals_.empty() || navigator_.searchExhausted() || (complete && eta > remaining * 1.2))) {
            considerFlightAt_ = zone.now + 0.7;
            flight_ = navigator_.flightPlan(start, goals_, zone, motion, energy, maximum,
                                            std::min(hardDeadline, Clock::now() + 4ms));
            if (const auto airborne = fly(me, start, zone, motion)) return {*airborne, "rotate-air"};
        }
        if (!target) {
            target = pickup(obs, start, threat, enemyInfos, zone, motion, route, complete);
        }

        // Active early loot patrol along the building-adjacent rim once inside nextZone (never idle!).
        const bool wantFarm = insideNext && !onRoof && !outside && !window.urgent && !hostileNow &&
            needsGear && zone.nextRadius >= 40.0;
        if (!target && wantFarm) {
            if (const auto localFarm = outerRimLoot(obs, start, enemyInfos, zone, motion, weapons)) {
                target = *localFarm;
                labelOverride = "farm-loot";
            } else {
                const double r = std::min(140.0, zone.nextRadius * 0.72);
                const double wpAngle = patrolIndex_ * (std::numbers::pi * 0.5);
                const Vec2 wpXY{zone.nextCenter[0] + r * std::cos(wpAngle),
                                zone.nextCenter[1] + r * std::sin(wpAngle)};
                if (distanceXY(start, wpXY) < 14.0) {
                    patrolIndex_ = (patrolIndex_ + 1) % 4;
                }
                const auto wp = navigator_.support(wpXY, motion, arena_.ground(wpXY[0], wpXY[1]) + motion.step + 0.1);
                if (wp && zone.inNext(*wp, motion.radius + 2.0)) {
                    auto pPlan = navigator_.plan(start, {*wp}, zone, motion, 9000 + patrolIndex_,
                                                 std::min(hardDeadline, tickStarted + 22ms));
                    if (!pPlan.route.empty()) {
                        target = pPlan.route.front();
                        labelOverride = "patrol-loot";
                    }
                }
            }
        }
        if (!target && !route.empty()) target = route.front();
        if (!target && !insideNext) {
            target = recover(start, zone, motion, std::min(hardDeadline, Clock::now() + 4ms), outside ? std::vector<Vec3>{} : enemies);
        }
        if (!outside && target && !enemies.empty() && !avoidsApproach(start, horizontal(*target), enemies)) {
            const auto escape = recover(start, zone, motion, std::min(hardDeadline, Clock::now() + 3ms), enemies);
            if (escape) target = escape;
        }
        monitorProgress(start, target, zone.now, navigator_.isSearching());
        const bool boost = grounded && target && !insideNext && (outside || eta + 3 > window.secondsUntilRisk) &&
            energy > 0.2 * maximum + motion.boostDrain * 0.2;
        auto command = groundCommand(start, target, me, zone, motion, boost, outside ? std::vector<Vec3>{} : enemies, window.urgent);
        if (window.urgent && (outside || zone.clearance(start, zone.now + 0.55) < 0) &&
            std::hypot(command.move[0], command.move[1]) < 0.12) {
            auto current = zone;
            current.nextCenter = current.center;
            current.nextRadius = current.radius;
            const auto forcedStep = recover(start, current, motion, std::min(hardDeadline, Clock::now() + 3ms),
                                            enemies, zone.nextCenter);
            if (forcedStep) {
                const auto lead = groundCommand(start, forcedStep, me, current, motion, boost, {}, true);
                const Vec2 projected{start[0] + lead.move[0] * 4, start[1] + lead.move[1] * 4};
                if (zone.clearance(projected, zone.now + 0.55) > zone.clearance(start, zone.now + 0.55) ||
                    distanceXY(projected, zone.nextCenter) < distanceXY(start, zone.nextCenter) - 0.1) {
                    command = lead;
                }
            }
        }
        return {command, outside ? "recover-zone" : labelOverride ? labelOverride : "rotate"};
    }

    // Refuse to leave cover or walk into enemy LOS just to greed a cube/pickup when another enemy is watching.
    bool lootPointSafe(const Vec3& start, const Vec3& itemPos, const std::vector<EnemyInfo>& enemyInfos,
                       const Motion& motion, double hpFrac) const {
        for (const auto& e : enemyInfos) {
            const double enemyToItem = distanceXY(e.pos, itemPos);
            if (enemyToItem > 175.0) continue;
            const bool itemExposed = wallCover(itemPos, e.pos, motion.height, arena_) < 3;
            if (!itemExposed) continue;
            const bool currentlyCovered = wallCover(start, e.pos, motion.height, arena_) == 3;
            if (currentlyCovered) return false;
            if (e.facing_us || e.charging || enemyToItem < 75.0 || regenMode_ || hpFrac < 0.70) {
                return false;
            }
        }
        return true;
    }

    std::optional<Vec3> outerRimLoot(const Json& obs, const Vec3& start, const std::vector<EnemyInfo>& enemyInfos,
                                     const Zone& zone, const Motion& motion, const Json& weapons) {
        const auto& me = obs.contains("self") && obs["self"].is_object() ? obs["self"] : Json::object();
        const double hpFrac = me.value("hp", 1400.0) / std::max(1.0, me.value("max_hp", 1400.0));
        const bool needsSecondary = countHeldWeapons(weapons) < 2;
        int slot1Prio = 0;
        if (weapons.is_array() && weapons.size() >= 2 && weapons[1].is_object()) {
            slot1Prio = weaponPriority(weapons[1].value("kind", std::string{}));
        }
        // 1. Visible weapon pickups in the current safe zone.
        if (const auto it = obs.find("pickups"); it != obs.end() && it->is_array()) {
            double bestScore = -infinity;
            std::optional<Vec3> best;
            for (const auto& item : *it) {
                if (!item.is_object()) continue;
                const auto kind = item.value("kind", std::string{});
                if (kind.empty() || kind == "gun" || hasWeaponKind(weapons, kind)) continue;
                const int prio = weaponPriority(kind);
                if (!needsSecondary && prio <= slot1Prio) continue;
                const auto pos = safeVec3(item, "pos", {infinity, infinity, 0.0});
                if (!std::isfinite(pos[0])) continue;
                const double dist = distanceXY(start, pos);
                if (dist > 70.0 || zone.clearance(horizontal(pos), zone.now + 8.0) <= motion.radius + 2.0) continue;
                const auto pt = navigator_.support(horizontal(pos), motion, pos[2] + 4.0);
                if (!pt || !navigator_.direct(start, *pt, zone, motion)) continue;
                if (!lootPointSafe(start, *pt, enemyInfos, motion, hpFrac)) continue;
                const double score = prio - dist * 1.2;
                if (score > bestScore) {
                    bestScore = score;
                    best = *pt;
                }
            }
            if (best) return best;
        }
        // 2. Visible power cubes (< 10) in the current safe zone.
        const double myCubes = me.value("cubes", 0.0);
        if (myCubes < cubeStatCap_ && threats_.can_collect_cubes(obs)) {
            if (const auto it = obs.find("cubes"); it != obs.end() && it->is_array()) {
                double bestDist = 45.0;
                std::optional<Vec3> best;
                for (const auto& item : *it) {
                    const auto pos = safeVec3(item, "pos", {infinity, infinity, 0.0});
                    if (!std::isfinite(pos[0])) continue;
                    const double dist = distanceXY(start, pos);
                    if (dist <= pickupRadius_ || dist >= bestDist) continue;
                    if (zone.clearance(horizontal(pos), zone.now + 6.0) <= motion.radius + 2.0) continue;
                    const auto pt = navigator_.support(horizontal(pos), motion, pos[2] + 4.0);
                    if (pt && navigator_.direct(start, *pt, zone, motion) &&
                        lootPointSafe(start, *pt, enemyInfos, motion, hpFrac)) {
                        bestDist = dist;
                        best = *pt;
                    }
                }
                if (best) return best;
            }
        }
        return std::nullopt;
    }

    std::optional<Vec3> pickup(const Json& obs, const Vec3& start, const std::optional<Threat>& threat,
                               const std::vector<EnemyInfo>& enemyInfos, const Zone& zone,
                               const Motion& motion, const Goals& route, bool complete) {
        const auto& me = obs.contains("self") && obs["self"].is_object() ? obs["self"] : Json::object();
        const double hpFrac = me.value("hp", 1400.0) / std::max(1.0, me.value("max_hp", 1400.0));
        const bool hostileThreat = threat && (threat->hostile || threat->damaged_recently || threat->laser_charging);
        if (hostileThreat || !me.value("in_zone", false) ||
            (!complete && !zone.inNext(start, motion.radius + 4))) {
            return {};
        }
        const double base = navigator_.pathTime(start, route, motion.walk);
        auto evaluateItem = [&](const Vec3& pos, double maxDetourSec) -> std::optional<Vec3> {
            const auto point = navigator_.support(horizontal(pos), motion, pos[2] + 4.0);
            if (!point || std::abs((*point)[2] - pos[2]) > 4.0 || !navigator_.direct(start, *point, zone, motion)) {
                return std::nullopt;
            }
            if (!lootPointSafe(start, *point, enemyInfos, motion, hpFrac)) {
                return std::nullopt;
            }
            if (!route.empty() && !navigator_.walkable(*point, route.front(), motion)) {
                // Allow direct detour if we are already safely inside next zone.
                if (!zone.inNext(*point, motion.radius)) {
                    return std::nullopt;
                }
                return point;
            }
            Goals path{*point};
            path.insert(path.end(), route.begin(), route.end());
            const double total = navigator_.pathTime(start, path, motion.walk);
            if (total - base <= maxDetourSec && navigator_.pathSafe(start, path, zone, motion.walk) &&
                zone.inNext(path.back(), motion.radius) &&
                (zone.inNext(start, motion.radius) || zone.now + total + 2 <= zone.end)) {
                return point;
            }
            return std::nullopt;
        };

        // 1. Prioritize weapon pickups (especially lightning!) when slot 1 is empty or upgradable.
        const auto& weapons = me.contains("weapons") && me["weapons"].is_array() ? me["weapons"] : Json::array();
        const bool needsSecondary = countHeldWeapons(weapons) < 2;
        int slot1Prio = 0;
        if (weapons.size() >= 2 && weapons[1].is_object()) {
            slot1Prio = weaponPriority(weapons[1].value("kind", std::string{}));
        }
        if (const auto it = obs.find("pickups"); it != obs.end() && it->is_array()) {
            double bestScore = -infinity;
            std::optional<Vec3> bestPickup;
            for (const auto& item : *it) {
                if (!item.is_object()) continue;
                const auto kind = item.value("kind", std::string{});
                if (kind.empty() || kind == "gun" || hasWeaponKind(weapons, kind)) continue;
                const int prio = weaponPriority(kind);
                if (!needsSecondary && prio <= slot1Prio) continue;
                const auto pos = safeVec3(item, "pos", {infinity, infinity, 0.0});
                if (!std::isfinite(pos[0])) continue;
                const double dist = distanceXY(start, pos);
                const double maxRange = needsSecondary ? 65.0 : 35.0;
                if (dist > maxRange || zone.clearance(horizontal(pos), zone.now) <= motion.radius) continue;
                const double score = prio - dist * 1.4;
                if (score > bestScore) {
                    if (const auto target = evaluateItem(pos, needsSecondary ? 6.5 : 3.5)) {
                        bestScore = score;
                        bestPickup = target;
                    }
                }
            }
            if (bestPickup) {
                return bestPickup;
            }
        }

        // 2. Collect safe power cubes up to stat_cap (10) so we gain +40% HP & damage without bloating hitbox.
        const double myCubes = me.value("cubes", 0.0);
        const bool wantCubes = myCubes < cubeStatCap_ || (myCubes == 14.0);
        if (wantCubes && threats_.can_collect_cubes(obs)) {
            const auto cubes = safe_cubes(obs, threat, pickupRadius_, [&](const Vec2& xy) {
                return zone.clearance(xy, zone.now) > motion.radius;
            });
            for (std::size_t i = 0; i < std::min<std::size_t>(4, cubes.size()); ++i) {
                const auto& cube = cubes[i].pos;
                if (distanceXY(start, cube) <= pickupRadius_ && std::abs(start[2] - cube[2]) <= 4) {
                    continue;
                }
                if (myCubes >= cubeStatCap_ && distanceXY(start, cube) > 10.0) {
                    continue;
                }
                if (const auto target = evaluateItem(cube, 3.5)) {
                    return target;
                }
            }
        }
        return {};
    }

    std::optional<Vec3> recover(const Vec3& start, const Zone& zone, const Motion& motion,
                                Clock::time_point deadline, const std::vector<Vec3>& enemies,
                                std::optional<Vec2> destination = {}) {
        const Vec2 center = destination.value_or(zone.nextRadius < zone.radius ? zone.nextCenter : zone.circle(zone.now + 0.8).first);
        const double yaw = std::atan2(center[1] - start[1], center[0] - start[0]);
        const double startDist = distanceXY(start, center);
        double bestSafeScore = -infinity;
        std::optional<Vec3> bestSafe;
        double bestFallbackScore = -infinity;
        std::optional<Vec3> bestFallback;
        for (const double length : {7.0, 3.5}) {
            for (const int offset : angles) {
                if (Clock::now() >= deadline) {
                    return bestSafe ? bestSafe : bestFallback;
                }
                const double angle = yaw + radians(offset);
                const Vec2 xy{start[0] + length * std::cos(angle), start[1] + length * std::sin(angle)};
                const double progress = startDist - distanceXY(xy, center);
                if (progress <= 0.1) {
                    continue;
                }
                const auto point = navigator_.support(xy, motion, start[2] + motion.step);
                if (!point || !navigator_.direct(start, *point, zone, motion)) {
                    continue;
                }
                if (avoidsApproach(start, xy, enemies)) {
                    if (progress > bestSafeScore) {
                        bestSafeScore = progress;
                        bestSafe = point;
                    }
                } else {
                    const double score = progress - 0.35 * approachPenalty(start, xy, enemies);
                    if (score > bestFallbackScore) {
                        bestFallbackScore = score;
                        bestFallback = point;
                    }
                }
            }
        }
        return bestSafe ? bestSafe : bestFallback;
    }

    void monitorProgress(const Vec3& start, const std::optional<Vec3>& target, double now, bool searching) {
        if (!target || distanceXY(start, *target) < 1) {
            progress_.reset();
            return;
        }
        if (!progress_ || distanceXY(progress_->target, *target) > 2) {
            progress_ = Progress{*target, start, now};
            return;
        }
        if (searching) {
            return;
        }
        if (now - progress_->time >= 1.5) {
            if (distanceXY(progress_->origin, *target) - distanceXY(start, *target) < 0.75) {
                navigator_.reset();
                goalDeadline_ = -infinity;
            }
            progress_ = Progress{*target, start, now};
        }
    }

    Command groundCommand(const Vec3& start, const std::optional<Vec3>& target, const Json& me,
                          const Zone& zone, const Motion& motion, bool boost,
                          const std::vector<Vec3>& enemies = {}, bool urgentZone = false) {
        const auto guarded = [&](const Vec2& move, bool sprint) {
            return navigator_.guard(move, sprint, me, zone, motion);
        };
        const auto acceptable = [&](const Vec2& move, bool sprint) {
            const Vec2 ahead{start[0] + 4 * move[0], start[1] + 4 * move[1]};
            return avoidsApproach(start, ahead, enemies) && guarded(move, sprint);
        };
        const Vec2 move = target ? direction(start, *target, std::min(1.0, distanceXY(start, *target) / 1.3)) : Vec2{};
        if (acceptable(move, boost)) {
            return {move, boost};
        }
        if (boost && acceptable(move, false)) {
            return {move};
        }

        if (target && distanceXY(start, *target) >= 1.0) {
            const double yaw = std::atan2(move[1], move[0]);
            const double startDist = distanceXY(start, *target);
            const double threatWeight = urgentZone ? 0.35 : 1.1;
            double bestScore = -infinity;
            std::optional<Command> bestCmd;
            for (const int offset : {0, 25, -25, 50, -50, 75, -75, 95, -95, 120, -120}) {
                const double angle = yaw + radians(offset);
                for (const double speed : {1.0, 0.55}) {
                    const Vec2 adjusted{speed * std::cos(angle), speed * std::sin(angle)};
                    const bool sprint = boost && speed > 0.9 && guarded(adjusted, true);
                    if (!sprint && !guarded(adjusted, false)) {
                        continue;
                    }
                    const Vec2 ahead{start[0] + adjusted[0] * 4.0, start[1] + adjusted[1] * 4.0};
                    const double progress = startDist - distanceXY(ahead, *target);
                    const double penalty = approachPenalty(start, ahead, enemies);
                    if (progress < -1.5 && penalty > 0.0) {
                        continue;
                    }
                    const double score = progress + 1.2 * speed - threatWeight * penalty;
                    if (score > bestScore) {
                        bestScore = score;
                        bestCmd = Command{adjusted, sprint};
                    }
                    break;
                }
            }
            if (bestCmd) {
                return *bestCmd;
            }
        }

        if (guarded({}, false)) {
            return {};
        }
        const Vec2 center = zone.circle(zone.now + 0.4).first;
        for (const double speed : {1.0, 0.66}) {
            const auto inward = direction(start, center, speed);
            if (boost && guarded(inward, true)) {
                return {inward, true};
            }
            if (guarded(inward, false)) {
                return {inward};
            }
        }
        return {};
    }

    std::optional<Command> fly(const Json& me, const Vec3& start, const Zone& zone, const Motion& motion) {
        if (!flight_) {
            return {};
        }
        auto& flight = *flight_;
        const double energy = me.value("energy", 0.0);
        const double maximum = std::max(1.0, me.value("max_energy", 100.0));
        const auto velocity = safeVec3(me, "vel");
        if (me.value("grounded", true)) {
            if (flight.airborne || energy <= 0.2 * maximum) {
                flight_.reset();
                return {};
            }
            const Vec2 toLanding = direction(start, flight.landing);
            const double velTowardLanding = velocity[0] * toLanding[0] + velocity[1] * toLanding[1];
            const double speedXY = std::hypot(velocity[0], velocity[1]);
            // Running jump-jet: launch immediately without braking to a dead stop when at takeoff and moving toward landing!
            const bool readyToLaunch = distanceXY(start, flight.takeoff) <= 1.8 &&
                (speedXY <= 2.5 || velTowardLanding >= -0.5);
            if (!readyToLaunch) {
                return groundCommand(start, flight.takeoff, me, zone, motion, false);
            }
            const Vec3 vertical{start[0], start[1], flight.apex};
            if (!arena_.flightSegment(start, vertical, motion.radius, motion.height) ||
                !zone.segment(start, start, zone.now, flight.climb) ||
                !zone.segment(start, flight.landing, zone.now + flight.climb, flight.seconds - flight.climb)) {
                flight_.reset();
                return {};
            }
            flight.airborne = true;
            return Command{{}, false, true};
        }
        if (!zone.segment(start, flight.landing, zone.now, distanceXY(start, flight.landing) / motion.walk + 0.3)) {
            for (const auto& xy : {zone.circle(zone.now + 0.6).first, horizontal(start)}) {
                const auto point = navigator_.support(xy, motion, start[2] + 0.1);
                if (point && zone.segment(start, *point, zone.now, distanceXY(start, *point) / motion.walk)) {
                    flight.landing = *point;
                    break;
                }
            }
        }
        const double offset = distanceXY(start, flight.landing);
        const double speed = std::min(1.0, std::sqrt(2 * motion.airAcceleration * std::max(0.0, offset - 0.35)) / motion.walk);
        const auto move = start[2] >= flight.apex - 0.4 ? direction(start, flight.landing, speed) : Vec2{};
        const bool jet = offset > 1.1 && energy > 0.2 * maximum + 0.2 * motion.jetDrain &&
            start[2] + velocity[2] * 0.16 < flight.apex + 0.4;
        if (navigator_.guard(move, jet, me, zone, motion, true)) {
            return Command{move, jet};
        }
        if (jet && navigator_.guard({}, true, me, zone, motion, true)) {
            return Command{{}, true};
        }
        return Command{};
    }

    Json config_;
    Arena arena_;
    Navigator navigator_;
    ThreatTracker threats_;
    double pickupRadius_;
    double lootPickupRadius_;
    double cubeStatCap_;
    std::optional<Flight> flight_;
    double considerFlightAt_ = -infinity;
    double considerRoofAt_ = -infinity;
    std::size_t patrolIndex_ = 0;
    Goals goals_;
    std::optional<std::pair<double, double>> rotationPhase_;
    std::optional<double> routeEstimate_;
    std::optional<Vec3> routeEstimatePos_;
    std::optional<RetreatIntent> retreat_;
    std::optional<Peek> peek_;
    std::optional<ActiveHop> hop_;
    std::optional<double> shelteredAt_;
    std::optional<double> scanHeading_;
    std::optional<int> lastTargetId_;
    std::optional<Vec3> combatAnchor_;
    double anchorSetAt_ = -infinity;
    bool regenMode_ = false;
    double strafeSign_ = 1.0;
    double nextStrafeFlipAt_ = -infinity;
    double nextPeekAt_ = -infinity;
    double nextHopAt_ = -infinity;
    double goalDeadline_ = -infinity;
    std::size_t routeId_ = 0;
    std::optional<Progress> progress_;
};

void send(const Json& message) {
    std::cout << message.dump() << '\n' << std::flush;
}

Json defaultAction() {
    return Json{
        {"move", {0.0, 0.0}},
        {"aim", {0.0, 0.0}},
        {"boost", false},
        {"jump", false},
        {"fire", {false, false}},
        {"reload", false},
    };
}

class Inbox {
public:
    bool readBatch() {
        if (!readSome()) {
            return false;
        }
        for (;;) {
            pollfd descriptor{STDIN_FILENO, POLLIN, 0};
            const int ready = ::poll(&descriptor, 1, 0);
            if (ready < 0 && errno == EINTR) {
                continue;
            }
            if (ready < 0) {
                throw std::system_error(errno, std::generic_category(), "poll stdin");
            }
            if (ready == 0 || !(descriptor.revents & (POLLIN | POLLHUP)) || !readSome()) {
                return true;
            }
        }
    }

    template <typename Handler>
    bool consume(Handler handle) {
        std::size_t start = 0;
        for (std::size_t end; (end = bytes_.find('\n', start)) != std::string::npos; start = end + 1) {
            auto message = Json::parse(bytes_.begin() + start, bytes_.begin() + end, nullptr, false);
            if (message.is_object() && !handle(std::move(message))) {
                return false;
            }
        }
        bytes_.erase(0, start);
        return true;
    }

private:
    bool readSome() {
        char block[32768];
        ssize_t size;
        do {
            size = ::read(STDIN_FILENO, block, sizeof(block));
        } while (size < 0 && errno == EINTR);
        if (size < 0) {
            throw std::system_error(errno, std::generic_category(), "read stdin");
        }
        bytes_.append(block, static_cast<std::size_t>(size));
        return size != 0;
    }

    std::string bytes_;
};

int run() {
    Inbox inbox;
    std::unique_ptr<Bot> bot;
    Json lastMove = defaultAction();
    while (inbox.readBatch()) {
        const auto batchStarted = Clock::now();
        std::optional<Json> latest;
        if (!inbox.consume([&](Json message) {
            const auto type = message.value("type", std::string{});
            if (type == "end") {
                return false;
            }
            if (type == "hello") {
                bot = std::make_unique<Bot>(message);
                lastMove = defaultAction();
                send({{"type", "ready"}, {"name", "osu-apex"}});
            } else if (type == "obs") {
                latest = std::move(message);
            }
            return true;
        })) {
            return 0;
        }
        if (!bot || !latest) {
            continue;
        }
        Json action = defaultAction();
        try {
            action = bot->act(*latest, batchStarted);
            lastMove = defaultAction();
            for (const auto* field : {"move", "aim", "boost"}) {
                if (action.contains(field)) {
                    lastMove[field] = action[field];
                }
            }
        } catch (const std::exception& error) {
            std::cerr << "Decision failed: " << error.what() << '\n';
            action = lastMove;
        }
        if (const auto tickIt = latest->find("tick"); tickIt != latest->end()) {
            action["tick"] = *tickIt;
        } else {
            action["tick"] = 0;
        }
        send(action);
    }
    return 0;
}

} // namespace
} // namespace mr

int main() {
    std::ios::sync_with_stdio(false);
    try {
        return mr::run();
    } catch (const std::exception& error) {
        std::cerr << "Bot stopped: " << error.what() << '\n';
        return 1;
    }
}
