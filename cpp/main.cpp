#include "arena.hpp"
#include "navigation.hpp"
#include "survival.hpp"
#include "tactics.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <map>
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

enum class Mode { Combat, Regen, Snowball, Rotate };

constexpr std::array modeNames{"COMBAT", "REGEN", "SNOWBALL", "ROTATE"};

struct Intent {
    Mode mode;
    Vec3 goal;
    const char* reason;
    bool sprint = false;
};

struct Frame {
    const Json& obs;
    const Json& me;
    const Json& stats;
    const Json& weapons;
    const Zone& zone;
    const Motion& motion;
    Vec3 pos;
    const std::optional<Threat>& threat;
    const std::vector<EnemyInfo>& enemies;
    RotationWindow window;
    Clock::time_point deadline;
    double hp, hpFraction, energy;
    bool inZone, grounded, onRoof;
    int exposed;
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
          lootPickupRadius_(config_.value("loot", Json::object()).value("pickup_radius", 4.0)),
          cubeStatCap_(config_.value("cubes", Json::object()).value("stat_cap", 10.0)) {}

    Json act(const Json& obs, Clock::time_point tickStarted) {
        const auto& me = obs.at("self");
        const auto& weapons = me.at("weapons");
        Json stats = config_.value("mech", Json::object());
        if (me.contains("stats")) stats.update(me.at("stats"));
        const Motion motion(stats);
        const Vec3 pos = safeVec3(me, "pos");
        const double now = obs.value("t", obs.value("tick", 0) * 0.05);
        const Zone zone(obs.at("zone"), now);
        if (routePhase_ != std::pair{zone.start, zone.end}) {
            routePhase_ = {zone.start, zone.end};
            routeEstimate_.reset();
            rotationGoal_.reset();
        }
        const auto deadline = tickStarted + 29ms;
        auto threat = threats_.update(obs, arena_, std::min(deadline, Clock::now() + 5ms));
        const auto enemies = threats_.visible_enemies(obs, arena_);
        const double hp = me.value("hp", 1400.0);
        const double maxHp = std::max(1.0, me.value("max_hp", 1400.0));
        const bool grounded = me.value("grounded", true);
        const bool onRoof = grounded && pos[2] > arena_.ground(pos[0], pos[1]) + 2.0 &&
                            std::abs(arena_.surface(pos[0], pos[1], pos[2] + 0.25) - pos[2]) < 0.3;
        int exposed = 0;
        int close = 0;
        for (const auto& enemy : enemies) {
            if (distanceXY(pos, enemy.pos) < 35.0) ++close;
            if (distanceXY(pos, enemy.pos) <= 280.0 && wallCover(pos, enemy.pos, motion.height, arena_) < 3) ++exposed;
        }
        const Frame frame{obs, me, stats, weapons, zone, motion, pos, threat, enemies,
                          rotationWindow(pos, zone, motion, routeEstimate_), deadline,
                          hp, hp / maxHp, me.value("energy", 0.0), me.value("in_zone", false),
                          grounded, onRoof, exposed};
        Intent intent = chooseIntent(frame);
        const Command command = execute(frame, intent);
        mode_ = intent.mode;
        Json action = {{"move", sanitizeMove(command.move)}, {"jump", command.jump}, {"boost", command.boost}};
        const bool charging = isLaserCharging(weapons);
        const bool mayFire = intent.mode != Mode::Regen || exposed > 0;
        const bool farm = intent.mode == Mode::Snowball;
        const bool chargeAllowed = mayFire && !frame.window.urgent &&
                                   (!command.boost || grounded) && !command.jump;
        std::optional<GunShot> shot;
        if (mayFire || charging) {
            shot = threats_.visible_shot(obs, arena_, threat, std::min(tickStarted + 39ms, Clock::now() + 8ms),
                                         farm, farm, chargeAllowed, lastTargetId_);
        }
        std::optional<Vec2> aim;
        if (shot) {
            aim = shot->aim;
            if (!shot->crate && !shot->npc && shot->target_id >= 0) lastTargetId_ = shot->target_id;
        } else if (const auto yaw = frontArmorYaw(pos, enemies, motion.height, arena_)) {
            aim = Vec2{*yaw, 0};
        } else if (threat && threat->position && !enemies.empty()) {
            aim = retreat_aim(obs, threat);
        } else if (const auto sound = threats_.sound_alert(now); sound && (sound->laser_charge || (onRoof && sound->near_jet))) {
            aim = Vec2{sound->bearing, 0};
        }
        action["aim"] = sanitizeAim(aim ? *aim : scanAim(me, command.move, now));
        const std::array<bool,2> fire = shot ? shot->fire : std::array{false, false};
        action["fire"] = fire;
        action["reload"] = false;
        if (!fire[0] && !fire[1] && !charging && !exposed && frame.inZone) {
            for (const auto& w : weapons) {
                if (!w.is_object() || w.value("kind", "") != std::string_view("gun") || w.value("reload", 0.0) > 0) continue;
                const auto ws = w.value("stats", Json::object());
                action["reload"] = w.value("ammo", 12) < std::min(8, ws.value("mag", 12)) &&
                                   frame.window.secondsUntilRisk - frame.window.estimatedTravel > ws.value("reload", 2.4);
            }
        }
        attachWeaponTake(obs, me, weapons, pos, charging, exposed, close, action);
        if (obs.contains("offer") && !obs.at("offer").is_null()) action["pick"] = chooseOfferPick(obs.at("offer"));
        if (config_.value("debug", Json::object()).value("max_text_chars", 0) >= 16)
            action["debug"] = {{"label", intent.reason}};
        if (trace_) {
            std::string weapon;
            bool fired = false;
            for (std::size_t i = 0; i < fire.size() && i < weapons.size(); ++i) {
                if (!weapons[i].is_object()) continue;
                const auto kind = weapons[i].value("kind", std::string{});
                const bool release = kind == "laser" && weapons[i].value("charge", 0.0) > 0 && !fire[i];
                if (fire[i] || release) {
                    if (!weapon.empty()) weapon += "+";
                    weapon += kind;
                    fired |= release || kind != "laser";
                }
            }
            std::cerr << Json{{"tick", obs.value("tick", 0)}, {"mode", modeNames[static_cast<int>(intent.mode)]},
                              {"movement_reason", intent.reason}, {"destination", intent.goal}, {"weapon", weapon},
                              {"airborne_shot", (command.jump || !grounded) && fired}, {"on_roof", onRoof},
                              {"exposed", exposed}}.dump() << '\n';
        }
        return action;
    }

private:
    using Goals = std::vector<Vec3>;

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

    Vec2 scanAim(const Json& me, const Vec2& move, double now) {
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

    Intent rotation(const Frame& f) {
        if (f.inZone && f.zone.inNext(f.pos, f.motion.radius + 3)) return {Mode::Rotate, f.pos, "rotate"};
        if (!rotationGoal_ || !f.zone.inNext(*rotationGoal_) || distanceXY(f.pos, *rotationGoal_) < 1.0) {
            auto options = rotationGoals(f.pos, f.zone, f.motion);
            rotationGoal_ = options.empty() ? std::optional<Vec3>{} : options.front();
        }
        const Vec3 goal = rotationGoal_.value_or(Vec3{f.zone.nextCenter[0], f.zone.nextCenter[1],
            arena_.ground(f.zone.nextCenter[0], f.zone.nextCenter[1])});
        return {Mode::Rotate, goal, f.inZone ? "rotate" : "recover-zone", f.window.urgent};
    }

    std::optional<Intent> snowball(const Frame& f) {
        const bool needCubes = f.me.value("cubes", 0.0) < cubeStatCap_;
        const bool needGear = countHeldWeapons(f.weapons) < 2;
        std::optional<Intent> best;
        double bestValue = 0;
        auto consider = [&](const Vec3& pos, double benefit, const char* label) {
            const double distance = distanceXY(f.pos, pos);
            const double eta = distance / std::max(1.0, f.motion.walk) + std::abs(pos[2] - f.pos[2]) / 12.0;
            const double value = benefit / (1.5 + eta);
            if (value <= bestValue || distance > 100 || f.zone.clearance(pos, f.zone.now + eta + 2) < f.motion.radius ||
                (failedGoal_ && f.zone.now < retryAt_ && distanceXY(pos, *failedGoal_) < 3)) return;
            const auto supported = navigator_.support(horizontal(pos), f.motion, pos[2] + 3.5);
            if (!supported || std::abs((*supported)[2] - pos[2]) > 4.0) return;
            for (const auto& e : f.enemies) {
                if (distanceXY(pos, e.pos) < 120 && wallCover(*supported, e.pos, f.motion.height, arena_) < 3) return;
            }
            bestValue = value;
            best = Intent{Mode::Snowball, *supported, label, false};
        };
        for (const auto& p : f.obs.at("pickups")) {
            const auto kind = p.value("kind", std::string{});
            const int held = f.weapons.size() > 1 && f.weapons[1].is_object()
                ? weaponPriority(f.weapons[1].value("kind", std::string{})) : 0;
            if (!hasWeaponKind(f.weapons, kind) && weaponPriority(kind) > held)
                consider(safeVec3(p, "pos"), weaponPriority(kind), "weapon");
        }
        if (needCubes) {
            for (const auto& c : f.obs.at("cubes")) consider(safeVec3(c,"pos"), 110, "cube");
            for (const auto& c : f.obs.at("crates")) consider(safeVec3(c,"pos"), 58, "crate");
            for (const auto& m : f.obs.at("mechs"))
                if (m.value("kind", "") == std::string_view("npc")) consider(safeVec3(m,"pos"), 65, "drone");
        }
        for (const auto& drop : f.obs.value("drops", Json::array())) {
            if (!needCubes && f.hpFraction >= 0.98) continue;
            if (drop.value("lands_in", 99.0) > 4.0 ||
                (drop.contains("holder") && drop["holder"].is_number() && drop["holder"] != f.me["id"])) continue;
            const auto p = safeVec3(drop, "pos");
            if (distanceXY(f.pos, p) <= drop.value("radius", 8.0) - 1 && drop.value("lands_in", 0.0) <= 0 &&
                f.enemies.empty() && f.zone.clearance(f.pos, f.zone.now + 4) > f.motion.radius)
                return Intent{Mode::Snowball, f.pos, "cap-drop"};
            consider(p, 190, "seek-drop");
        }
        if (best || (!needGear && !needCubes)) return best;
        const auto roofs = arena_.nearbyRooftops(f.pos, 85, -4.0, 36.0, f.motion.radius, f.motion.height);
        for (const auto& roof : roofs) {
            if (distanceXY(f.pos, roof) < 5 && std::abs(f.pos[2]-roof[2]) < 1) {
                searched_[roof] = f.zone.now;
                continue;
            }
            const auto seen = searched_.find(roof);
            if (seen != searched_.end() && f.zone.now - seen->second < 45.0) continue;
            consider(roof, 25, "search-roof");
        }
        return best;
    }

    Intent chooseIntent(const Frame& f) {
        const bool opponent = f.threat && f.threat->position && !f.threat->is_npc;
        const bool advantage = opponent && f.enemies.size() == 1 && f.exposed == 1 &&
            f.threat->hp < f.hp * 0.85 && f.hpFraction >= 0.25;
        const bool needsRest = f.hpFraction < (mode_ == Mode::Regen ? 0.98 : (f.exposed ? 0.42 : 0.80));
        if (!f.inZone || f.window.urgent) return rotation(f);
        if (needsRest && !advantage) {
            if (opponent && f.exposed) {
                std::vector<Vec3> positions;
                for (const auto& enemy : f.enemies) positions.push_back(enemy.pos);
                if (auto cover = planRetreat(f.pos, *f.threat, positions, f.zone, f.motion, arena_, navigator_,
                                             std::min(f.deadline, Clock::now() + 4ms)))
                    return {Mode::Regen, cover->target, "seek-cover", true};
            }
            return {Mode::Regen, f.pos, "regen-cover"};
        }
        if (opponent && (f.exposed || f.threat->hostile)) {
            CombatContext ctx;
            ctx.hp = f.hp;
            ctx.maxHp = f.me.value("max_hp", 1400.0);
            ctx.energy = f.energy;
            ctx.muzzleHeight = f.stats.value("muzzle_height", 7.0);
            ctx.hasLightning = hasWeaponKind(f.weapons, "lightning");
            ctx.hasLaser = hasWeaponKind(f.weapons, "laser");
            ctx.hasMortar = hasWeaponKind(f.weapons, "mortar");
            ctx.reloading = isGunReloading(f.weapons) && !hasReadySecondary(f.weapons, f.energy);
            ctx.laserCharging = isLaserCharging(f.weapons);
            ctx.onRoof = f.onRoof;
            ctx.velocity = horizontal(safeVec3(f.me, "vel"));
            ctx.anchor = f.pos;
            if (auto move = planCombatPosition(f.pos, *f.threat, f.enemies, ctx, f.zone, f.motion,
                                               arena_, navigator_, std::min(f.deadline, Clock::now() + 4ms)))
                return {Mode::Combat, move->target, move->label, move->boost};
            return {Mode::Combat, f.pos, "engage"};
        }
        if (auto farm = snowball(f)) return *farm;
        return rotation(f);
    }
    Command execute(const Frame& f, Intent& intent) {
        if (flight_) {
            if (!flight_->airborne && (flightOwner_.mode != intent.mode || distanceXY(intent.goal, flightOwner_.goal) > 5))
                flight_.reset();
            if (flight_) {
                if (auto command = fly(f.me, f.pos, f.zone, f.motion)) {
                    intent = flightOwner_;
                    intent.reason = "flight";
                    return *command;
                }
            }
        }
        if (hop_ && !f.grounded && f.zone.now < hopEnds_) {
            intent.goal = hop_->landing;
            intent.reason = "air-hop";
            const Vec2 steer = f.zone.now < steerAt_ ? Vec2{} : hop_->launch;
            return {navigator_.guard(steer, false, f.me, f.zone, f.motion) ? steer : Vec2{}};
        }
        hop_.reset();
        if (const auto dodge = threats_.projectile_dodge(f.obs, arena_); dodge && dodge->urgent_mortar &&
            f.grounded && navigator_.guard(dodge->direction, false, f.me, f.zone, f.motion)) {
            intent.reason = "dodge-mortar";
            intent.goal = {f.pos[0] + 4 * dodge->direction[0], f.pos[1] + 4 * dodge->direction[1], f.pos[2]};
            return {dodge->direction};
        }
        if (f.grounded && f.inZone && f.threat && intent.mode == Mode::Combat && f.zone.now >= nextHopAt_ &&
            (f.threat->laser_charging || f.threat->damaged_recently)) {
            std::vector<Vec3> enemies;
            for (const auto& e : f.enemies) enemies.push_back(e.pos);
            hop_ = planDefensiveHop(f.me, *f.threat, enemies, f.zone, f.motion, arena_, navigator_,
                                    true, false, std::min(f.deadline, Clock::now() + 3ms));
            if (hop_) {
                nextHopAt_ = f.zone.now + 2.4;
                hopEnds_ = f.zone.now + hop_->duration + 0.35;
                steerAt_ = f.zone.now + hop_->steerAfter;
                intent.goal = hop_->landing;
                intent.reason = "cover-hop";
                return {hop_->steerAfter > 0 ? Vec2{} : hop_->launch, false, true};
            }
        }
        if (distanceXY(f.pos, intent.goal) < 0.55 && std::abs(f.pos[2] - intent.goal[2]) < 1) return {};
        const bool sprint = intent.sprint && f.grounded && f.energy > 18 && !isLaserCharging(f.weapons);
        if (navigator_.direct(f.pos, intent.goal, f.zone, f.motion)) {
            if (intent.mode == Mode::Rotate) routeEstimate_ = distanceXY(f.pos, intent.goal) / std::max(1.0, f.motion.walk);
            return groundCommand(f.pos, intent.goal, f.me, f.zone, f.motion, sprint);
        }
        if (!routeGoal_ || distanceXY(*routeGoal_, intent.goal) > 1 || std::abs((*routeGoal_)[2] - intent.goal[2]) > 1 ||
            intent.mode != routeMode_) {
            routeGoal_ = intent.goal;
            routeMode_ = intent.mode;
            ++routeId_;
        }
        const auto route = navigator_.plan(f.pos, {intent.goal}, f.zone, f.motion, routeId_,
                                          std::min(f.deadline, Clock::now() + 5ms));
        const double walkTime = route.complete ? navigator_.pathTime(f.pos, route.route, f.motion.walk) : infinity;
        if (intent.mode == Mode::Rotate && route.complete) routeEstimate_ = walkTime;
        if (f.grounded && f.zone.now >= considerFlightAt_ && f.energy > 20 &&
            (intent.goal[2] > f.pos[2] + f.motion.step || intent.mode == Mode::Rotate)) {
            considerFlightAt_ = f.zone.now + 0.45;
            if (intent.goal[2] > f.pos[2] + f.motion.step)
                flight_ = navigator_.flightToRoof(f.pos, intent.goal, f.zone, f.motion, f.energy, f.me.value("max_energy",100.0), f.deadline);
            else
                flight_ = navigator_.flightPlan(f.pos, {intent.goal}, f.zone, f.motion, f.energy, f.me.value("max_energy",100.0), f.deadline);
            if (flight_ && flight_->seconds + distanceXY(f.pos, flight_->takeoff) / std::max(1.0, f.motion.walk) < walkTime) {
                flightOwner_ = intent;
                flightOwner_.goal = flight_->landing;
                if (auto cmd = fly(f.me, f.pos, f.zone, f.motion)) {
                    intent.reason = "launch";
                    return *cmd;
                }
            }
            flight_.reset();
        }
        if (!route.route.empty()) return groundCommand(f.pos, route.route.front(), f.me, f.zone, f.motion, sprint);
        if (navigator_.searchExhausted()) {
            failedGoal_ = intent.goal;
            retryAt_ = f.zone.now + 5.0;
            if (intent.mode == Mode::Rotate) rotationGoal_.reset();
            intent.reason = "unreachable";
        }
        if (intent.mode == Mode::Rotate)
            return groundCommand(f.pos, intent.goal, f.me, f.zone, f.motion, sprint, {}, true);
        return {};
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
            if (distanceXY(start, flight.takeoff) > 1.0 || std::hypot(velocity[0], velocity[1]) > 2) {
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
    double lootPickupRadius_;
    double cubeStatCap_;
    bool trace_ = std::getenv("MECH_TRACE") && std::string_view(std::getenv("MECH_TRACE")) == "1";
    Mode mode_ = Mode::Rotate;
    std::optional<Flight> flight_;
    Intent flightOwner_{Mode::Rotate, {}, "flight"};
    std::optional<DefensiveHop> hop_;
    double hopEnds_ = 0, steerAt_ = 0, nextHopAt_ = 0, considerFlightAt_ = 0;
    std::optional<int> lastTargetId_;
    std::optional<double> scanHeading_, routeEstimate_;
    std::optional<Vec3> routeGoal_, rotationGoal_, failedGoal_;
    std::pair<double,double> routePhase_{-1,-1};
    Mode routeMode_ = Mode::Rotate;
    std::size_t routeId_ = 0;
    std::map<Vec3,double> searched_;
    double retryAt_ = 0;
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
                send({{"type", "ready"}, {"name", "mech-vnext"}});
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
