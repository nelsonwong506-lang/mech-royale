#include "brain.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string_view>

namespace mr {
namespace {

constexpr double kTau = 2.0 * std::numbers::pi;
constexpr double kLat = 0.075;
constexpr double kInf = std::numeric_limits<double>::infinity();

double angDiff(double a, double b) {
    return std::remainder(a - b, kTau);
}

bool holdsKind(const std::vector<std::string>& list, std::string_view kind) {
    return std::any_of(list.begin(), list.end(), [&](const std::string& k) { return k == kind; });
}

int findWeaponSlot(const std::array<std::optional<WeaponInfo>, 2>& ws, std::string_view kind) {
    for (int i = 0; i < 2; ++i) {
        if (ws[static_cast<std::size_t>(i)] && ws[static_cast<std::size_t>(i)]->kind == kind) {
            return i;
        }
    }
    return -1;
}

const WeaponInfo* getWeapon(const std::array<std::optional<WeaponInfo>, 2>& ws, std::string_view kind) {
    const int idx = findWeaponSlot(ws, kind);
    return idx >= 0 ? &(*ws[static_cast<std::size_t>(idx)]) : nullptr;
}

std::array<std::optional<WeaponInfo>, 2> parseWeapons(const Json& me) {
    std::array<std::optional<WeaponInfo>, 2> out;
    const auto it = me.find("weapons");
    if (it == me.end() || !it->is_array()) return out;
    for (std::size_t i = 0; i < std::min<std::size_t>(2, it->size()); ++i) {
        const auto& w = (*it)[i];
        if (!w.is_object()) continue;
        WeaponInfo wi;
        wi.kind = w.value("kind", std::string{});
        wi.cooldown = finiteOr(w.value("cooldown", 0.0), 0.0);
        wi.ammo = finiteOr(w.value("ammo", 0.0), 0.0);
        wi.reload = finiteOr(w.value("reload", 0.0), 0.0);
        wi.charge = finiteOr(w.value("charge", 0.0), 0.0);
        const auto& st = w.contains("stats") && w["stats"].is_object() ? w["stats"] : Json::object();
        if (wi.kind == "gun") {
            wi.damage = finiteOr(st.value("damage", 45.0), 45.0);
            wi.speed = finiteOr(st.value("speed", 252.0), 252.0);
            wi.range = finiteOr(st.value("range", 230.0), 230.0);
            wi.mag = finiteOr(st.value("mag", 12.0), 12.0);
        } else if (wi.kind == "laser") {
            wi.damage = finiteOr(st.value("damage", 315.0), 315.0);
            wi.range = finiteOr(st.value("range", 280.0), 280.0);
            wi.fullCharge = std::max(0.15, finiteOr(st.value("charge", 0.75), 0.75));
            wi.minCharge = finiteOr(st.value("min_charge", 0.15), 0.15);
            wi.minFrac = finiteOr(st.value("min_frac", 0.35), 0.35);
            wi.holdS = finiteOr(st.value("hold_s", 2.0), 2.0);
            wi.cookoff = finiteOr(st.value("cookoff", 0.6), 0.6);
        } else if (wi.kind == "lightning") {
            wi.damage = finiteOr(st.value("damage", 60.0), 60.0);
            wi.range = finiteOr(st.value("range", 45.0), 45.0);
            wi.coneDeg = finiteOr(st.value("cone_deg", 30.0), 30.0);
            wi.energyCost = finiteOr(st.value("energy", 3.0), 3.0);
        } else if (wi.kind == "mortar") {
            wi.damage = finiteOr(st.value("damage", 200.0), 200.0);
            wi.speed = finiteOr(st.value("speed", 72.0), 72.0);
            wi.gravity = finiteOr(st.value("gravity", 22.0), 22.0);
            wi.splashRadius = finiteOr(st.value("splash_radius", 7.0), 7.0);
            wi.range = finiteOr(st.value("range", wi.speed * wi.speed / std::max(1.0, wi.gravity)), 235.0);
        }
        out[i] = std::move(wi);
    }
    return out;
}

}  // namespace

BrainParams Brain::defaultParams() {
    BrainParams p;
    p.retreatHP = 0.38;
    p.recoverHP = 0.80;
    p.gunRange = 85.0;
    p.lightRange = 28.0;
    p.laserRange = 85.0;
    p.engageDist = 60.0;
    p.farmNPC = true;
    p.explore = false;
    p.ghost = true;
    p.homeExpo = 120.0;
    p.lootRadius = 90.0;
    p.crateUntil = 150.0;
    p.dodge = true;
    p.noMortar = true;
    p.peek = true;
    p.laserFirst = true;
    p.crowdHide = true;
    return p;
}

BrainParams Brain::squadParams(BrainParams p) {
    p.ghost = false;
    p.peek = false;
    p.laserFirst = true;
    p.noMortar = true;
    p.laserRange = 120.0;
    p.lootRadius = 150.0;
    p.crateUntil = 1e9;
    p.stick = 200.0;
    p.hpW = 0.18;
    p.lowBonus = 200.0;
    p.maxTarget = 200.0;
    p.loseRule = true;
    p.fightLoot = 70.0;
    p.farmHP = 0.50;
    p.laserSeek = 240.0;
    p.laserUp = true;
    p.lethal = true;
    p.lethalK = 0.70;
    p.finish = 20.0;
    p.finishHP = 0.38;
    return p;
}

Brain::Brain(const Json& hello)
    : config_(hello.is_object() ? hello.value("config", Json::object()) : Json::object()),
      arena_(hello.at("map").at("dir").get<std::string>(),
             hello.at("map").value("files", std::vector<std::string>{})),
      nav_(arena_, 2.6, 2.0),
      rng_(static_cast<std::uint32_t>(hello.value("seed", 0) != 0 ? hello.value("seed", 1) : hello.value("slot", 0) + 1)),
      baseP_(defaultParams()),
      p_(baseP_) {
    const auto& mech = config_.value("mech", Json::object());
    cfgMechRadius_ = finiteOr(mech.value("radius", 2.5), 2.5);
    cfgMechHeight_ = finiteOr(mech.value("height", 9.0), 9.0);
    cfgMechEye_ = finiteOr(mech.value("eye_height", 8.0), 8.0);
    cfgMechMuzzle_ = finiteOr(mech.value("muzzle_height", 7.0), 7.0);
    cfgFrontArcDeg_ = finiteOr(mech.value("front_arc_deg", 180.0), 180.0);
    cfgRearMult_ = finiteOr(mech.value("rear_mult", 1.5), 1.5);

    const auto& npc = config_.value("npc", Json::object());
    cfgNpcRadius_ = finiteOr(npc.value("radius", 1.8), 1.8);
    cfgNpcHeight_ = finiteOr(npc.value("height", 3.5), 3.5);

    const auto& cr = config_.value("crates", Json::object());
    cfgCrateRadius_ = finiteOr(cr.value("radius", 1.4), 1.4);
    cfgCrateHeight_ = finiteOr(cr.value("height", 2.4), 2.4);

    const auto& cu = config_.value("cubes", Json::object());
    cfgSizePerCube_ = finiteOr(cu.value("size_per_cube", 0.05), 0.05);
    cfgDmgPerCube_ = finiteOr(cu.value("damage_per_cube", 0.04), 0.04);
    cfgCubeStatCap_ = cu.value("stat_cap", 10);

    const auto& lt = config_.value("loot", Json::object());
    cfgLootPickupRadius_ = finiteOr(lt.value("pickup_radius", 4.0), 4.0);

    const auto& weps = config_.value("weapons", Json::object());
    const auto& mort = weps.value("mortar", Json::object());
    cfgMortarGravity_ = finiteOr(mort.value("gravity", 22.0), 22.0);
    cfgMortarSplash_ = finiteOr(mort.value("splash_radius", 7.0), 7.0);

    cfgDebugMaxChars_ = config_.value("debug", Json::object()).value("max_text_chars", 0);
}

double Brain::scaleOf(int cubes) const {
    return 1.0 + cfgSizePerCube_ * static_cast<double>(std::max(0, cubes));
}

Vec3 Brain::centerOf(const MechTrack& t) const {
    const double h = (t.kind == "npc") ? cfgNpcHeight_ : cfgMechHeight_ * scaleOf(t.cubes);
    return {t.pos[0], t.pos[1], t.pos[2] + h * 0.5};
}

Vec3 Brain::eyeOf(const MechTrack& t) const {
    return {t.pos[0], t.pos[1], t.pos[2] + cfgMechEye_ * scaleOf(t.cubes)};
}

bool Brain::showsBack(const MechTrack& t, double x, double y) const {
    const double arc = cfgFrontArcDeg_ > 0.0 ? cfgFrontArcDeg_ : 180.0;
    const double toMe = std::atan2(y - t.pos[1], x - t.pos[0]);
    return std::abs(angDiff(t.aim[0], toMe)) > radians(arc * 0.5) + 0.1;
}

bool Brain::wantPickup(const std::array<std::optional<WeaponInfo>, 2>& ws, const std::string& kind) const {
    if (kind == "gun") {
        return !ws[0].has_value();
    }
    if (findWeaponSlot(ws, kind) >= 0) {
        return false;
    }
    if (kind == "mortar" && p_.noMortar) {
        return false;
    }
    auto rankOf = [&](std::string_view k) -> int {
        if (k == "laser") return p_.laserFirst ? 4 : 2;
        if (k == "lightning") return 3;
        if (k == "mortar") return 1;
        return 0;
    };
    const int cur = ws[1] ? rankOf(ws[1]->kind) : 0;
    return rankOf(kind) > cur;
}

int Brain::pickUpgrade(const Json& offer) const {
    const Json* choices = nullptr;
    if (offer.is_array() && !offer.empty()) {
        choices = &offer;
    } else if (offer.is_object()) {
        for (const char* k : {"choices", "options", "items", "upgrades"}) {
            const auto it = offer.find(k);
            if (it != offer.end() && it->is_array() && !it->empty()) {
                choices = &(*it);
                break;
            }
        }
    }
    if (!choices || choices->empty()) return 0;

    static constexpr std::array<std::string_view, 22> kOrderDefault{
        "plating", "armor", "gun_damage", "nanites", "gun_rate",
        "lightning_efficiency", "lightning_range", "laser_damage",
        "servos", "laser_charge", "gun_mag", "gun_velocity", "gyros",
        "lightning_chain", "reactor", "capacitor",
        "mortar_reload", "mortar_splash", "mortar_velocity",
        "sensors", "jets", "laser_pierce"
    };
    static constexpr std::array<std::string_view, 22> kOrderLaser{
        "plating", "armor", "laser_damage", "laser_charge", "nanites",
        "gun_damage", "gun_rate", "lightning_efficiency", "lightning_range",
        "servos", "gun_mag", "gun_velocity", "gyros",
        "lightning_chain", "reactor", "capacitor",
        "mortar_reload", "mortar_splash", "mortar_velocity",
        "sensors", "jets", "laser_pierce"
    };
    const auto& order = p_.laserUp ? kOrderLaser : kOrderDefault;

    int best = 0;
    std::size_t bestRank = order.size() + 1;
    for (std::size_t i = 0; i < choices->size(); ++i) {
        const auto& c = (*choices)[i];
        std::string_view id;
        if (c.is_object() && c.contains("id") && c["id"].is_string()) {
            id = c["id"].get_ref<const std::string&>();
        }
        std::size_t rank = order.size();
        for (std::size_t r = 0; r < order.size(); ++r) {
            if (order[r] == id) {
                rank = r;
                break;
            }
        }
        if (rank < bestRank) {
            best = static_cast<int>(i);
            bestRank = rank;
        }
    }
    return best;
}

void Brain::observe(const Json& obs, const Json& me, const std::array<std::optional<WeaponInfo>, 2>& ws) {
    (void)ws;
    const Vec3 pos = safeVec3(me, "pos");
    const Vec3 vel = safeVec3(me, "vel");
    const auto& st = me.contains("stats") && me["stats"].is_object() ? me["stats"] : Json::object();
    const double eyeH = finiteOr(st.value("eye_height", cfgMechEye_), cfgMechEye_);
    const double muzH = finiteOr(st.value("muzzle_height", cfgMechMuzzle_), cfgMechMuzzle_);
    const Vec3 eye{pos[0], pos[1], pos[2] + eyeH};
    const Vec3 muzzle{pos[0], pos[1], pos[2] + muzH};
    const int selfId = me.contains("id") && me["id"].is_number() ? static_cast<int>(me["id"].get<double>()) : -1;

    if (const auto it = obs.find("mechs"); it != obs.end() && it->is_array()) {
        for (const auto& m : *it) {
            if (!m.is_object() || !m.contains("id") || !m["id"].is_number()) continue;
            const int id = static_cast<int>(m["id"].get<double>());
            if (id == selfId) continue;
            const std::string kind = m.value("kind", std::string{"mech"});
            auto& tab = (kind == "npc") ? npcs_ : tracks_;
            auto& t = tab[id];
            t.id = id;
            t.kind = kind;
            t.pos = safeVec3(m, "pos");
            t.vel = safeVec3(m, "vel");
            t.aim = safeVec2(m, "aim");
            t.hp = finiteOr(m.value("hp", kind == "npc" ? 350.0 : 1400.0), 1400.0);
            t.maxHp = std::max(1.0, finiteOr(m.value("max_hp", kind == "npc" ? 350.0 : 1400.0), 1400.0));
            t.cubes = static_cast<int>(finiteOr(m.value("cubes", 0.0), 0.0));
            t.charging = m.value("charging", false);
            t.weapons.clear();
            if (m.contains("weapons") && m["weapons"].is_array()) {
                for (const auto& wk : m["weapons"]) {
                    if (wk.is_string()) t.weapons.push_back(wk.get<std::string>());
                }
            }
            t.seen = now_;
            const double dx = t.pos[0] - pos[0], dy = t.pos[1] - pos[1];
            t.dist = std::hypot(dx, dy);
            const Vec3 c = centerOf(t);
            t.los = !arena_.blocksLine(muzzle, c);
            if (!t.los && kind == "mech") {
                const double h = cfgMechHeight_ * scaleOf(t.cubes);
                t.los = !arena_.blocksLine(muzzle, {t.pos[0], t.pos[1], t.pos[2] + h * 0.85});
            }
            if (t.dist > 1.0) {
                t.closing = -((t.vel[0] - vel[0]) * dx + (t.vel[1] - vel[1]) * dy) / t.dist;
            } else {
                t.closing = 0.0;
            }
        }
    }

    for (auto it = tracks_.begin(); it != tracks_.end(); ) {
        if (now_ - it->second.seen > 20.0) it = tracks_.erase(it);
        else ++it;
    }
    for (auto it = npcs_.begin(); it != npcs_.end(); ) {
        if (now_ - it->second.seen > 120.0) it = npcs_.erase(it);
        else ++it;
    }

    auto updateItems = [&](std::unordered_map<int, KnownItem>& tab, const char* fieldName) {
        if (const auto it = obs.find(fieldName); it != obs.end() && it->is_array()) {
            for (const auto& item : *it) {
                if (!item.is_object() || !item.contains("id") || !item["id"].is_number()) continue;
                const int id = static_cast<int>(item["id"].get<double>());
                auto& k = tab[id];
                k.id = id;
                k.kind = item.value("kind", std::string{});
                k.pos = safeVec3(item, "pos");
                k.hp = finiteOr(item.value("hp", 0.0), 0.0);
                k.seen = now_;
            }
        }
        for (auto it = tab.begin(); it != tab.end(); ) {
            if (it->second.seen == now_) {
                ++it;
                continue;
            }
            const double d = distanceXY(it->second.pos, pos);
            if (d < 40.0 && now_ - it->second.seen > 0.35 &&
                !arena_.blocksLine(eye, {it->second.pos[0], it->second.pos[1], it->second.pos[2] + 1.5})) {
                it = tab.erase(it);
            } else {
                ++it;
            }
        }
    };
    updateItems(crates_, "crates");
    updateItems(cubes_, "cubes");
    updateItems(pickups_, "pickups");

    if (const auto evIt = obs.find("events"); evIt != obs.end() && evIt->is_array()) {
        for (const auto& e : *evIt) {
            if (!e.is_object()) continue;
            const std::string type = e.value("type", std::string{});
            if (type == "damaged") {
                const std::string wep = e.value("weapon", std::string{});
                if (wep == "npc" && e.contains("bearing") && e["bearing"].is_number()) {
                    npcHurt_ = now_;
                    npcBearing_ = finiteOr(e["bearing"].get<double>(), 0.0);
                }
                if (e.contains("by") && e["by"].is_number()) {
                    const int byId = static_cast<int>(e["by"].get<double>());
                    lastHurt_ = now_;
                    if (e.contains("bearing") && e["bearing"].is_number()) {
                        hurtBearing_ = finiteOr(e["bearing"].get<double>(), 0.0);
                    }
                    if (auto ti = tracks_.find(byId); ti != tracks_.end()) {
                        ti->second.hitMe = now_;
                    }
                    if (auto ni = npcs_.find(byId); ni != npcs_.end()) {
                        ni->second.hitMe = now_;
                    }
                }
            } else if (type == "effect") {
                if (e.value("effect", "") == std::string_view("short")) {
                    shortUntil_ = std::max(shortUntil_, now_ + 2.05);
                }
            } else if (type == "kill") {
                ++kills_;
                if (e.contains("target") && e["target"].is_number()) {
                    tracks_.erase(static_cast<int>(e["target"].get<double>()));
                }
            }
        }
    }

    // Automatic Practice (1v15 Squad) vs Final (16-way FFA) mode detection!
    const int alive = obs.value("alive", 16);
    if (!initDone_) {
        initDone_ = true;
        startAlive_ = alive;
    }
    const int otherDead = startAlive_ - alive - kills_;
    if (otherDead > 0) {
        ++otherDeadTicks_;
    } else {
        otherDeadTicks_ = 0;
    }
    if (otherDeadTicks_ >= 5) {
        notPractice_ = true;
    }
    const bool prac = !notPractice_ && now_ > 20.0;
    if (prac != pracOn_) {
        pracOn_ = prac;
        p_ = prac ? squadParams(baseP_) : baseP_;
    }

    // Zone phase anchor tracking
    if (const auto zit = obs.find("zone"); zit != obs.end() && zit->is_object()) {
        const auto& z = *zit;
        const double ss = finiteOr(z.value("shrink_start", 0.0), 0.0);
        const Vec2 c = safeVec2(z, "center");
        const double r = finiteOr(z.value("radius", 560.0), 560.0);
        if (holdStart_ != ss || now_ <= ss) {
            holdStart_ = ss;
            hold_ = {c[0], c[1], r};
        }
    }
}

ZoneCircle Brain::zoneAt(const Json& z, double t) const {
    const double ss = finiteOr(z.value("shrink_start", 0.0), 0.0);
    const double se = finiteOr(z.value("shrink_end", 0.0), 0.0);
    if (t <= ss || se <= ss) {
        return hold_;
    }
    const Vec2 nc = safeVec2(z, "next_center", {hold_.x, hold_.y});
    const double nr = finiteOr(z.value("next_radius", hold_.r), hold_.r);
    const double f = std::clamp((t - ss) / (se - ss), 0.0, 1.0);
    return {hold_.x + (nc[0] - hold_.x) * f,
            hold_.y + (nc[1] - hold_.y) * f,
            hold_.r + (nr - hold_.r) * f};
}

ZoneCircle Brain::curZone(const Json& z) const {
    const Vec2 c = safeVec2(z, "center");
    const double r = finiteOr(z.value("radius", 560.0), 560.0);
    const ZoneCircle live{c[0], c[1], r};
    const double ss = finiteOr(z.value("shrink_start", 0.0), 0.0);
    if (now_ <= ss) return live;
    if (std::abs(live.r - hold_.r) > 0.01 || std::abs(live.x - hold_.x) > 0.01) {
        return live;
    }
    return zoneAt(z, now_);
}

double Brain::zoneDeadline(const Json& z, double x, double y, double pad) const {
    const Vec2 nc = safeVec2(z, "next_center");
    const double nr = finiteOr(z.value("next_radius", 0.0), 0.0);
    if (std::hypot(x - nc[0], y - nc[1]) <= nr - pad) {
        return kInf;
    }
    const double ss = finiteOr(z.value("shrink_start", 0.0), 0.0);
    const double se = finiteOr(z.value("shrink_end", 0.0), 0.0);
    for (int k = 0; k <= 24; ++k) {
        const double t = ss + (se - ss) * (static_cast<double>(k) / 24.0);
        const ZoneCircle c = zoneAt(z, t);
        if (std::hypot(x - c.x, y - c.y) > c.r - pad) {
            return t;
        }
    }
    return se;
}

double Brain::zonePenalty(const Json& z, double x, double y) const {
    const ZoneCircle c = curZone(z);
    double pen = 0.0;
    if (const double d = std::hypot(x - c.x, y - c.y) - c.r; d > -5.0) {
        pen += 150.0 + 5.0 * (d + 5.0);
    }
    const Vec2 nc = safeVec2(z, "next_center");
    const double nr = finiteOr(z.value("next_radius", 0.0), 0.0);
    if (const double dn = std::hypot(x - nc[0], y - nc[1]) - nr; dn > -5.0) {
        const double ss = finiteOr(z.value("shrink_start", 0.0), 0.0);
        const double soon = std::clamp(1.0 - (ss - now_) / 25.0, 0.1, 1.0);
        pen += soon * (25.0 + 0.8 * (dn + 5.0));
    }
    return pen;
}

Vec2 Brain::zoneGoal(const Json& z, double x, double y) const {
    const Vec2 nc = safeVec2(z, "next_center");
    const double nr = finiteOr(z.value("next_radius", 0.0), 0.0);
    if (nr <= 30.0) {
        return nc;
    }
    const double margin = std::clamp(0.3 * nr, 3.0, 30.0);
    const double dx = x - nc[0], dy = y - nc[1];
    const double d = std::hypot(dx, dy);
    if (d < 1.0) return {x, y};
    const double r = std::max(0.0, nr - margin - 5.0);
    if (d <= r) return {x, y};
    return {nc[0] + dx / d * r, nc[1] + dy / d * r};
}

bool Brain::runEscape(const Json& me, double& mx, double& my, bool& boost, bool& jump) {
    const Vec3 pos = safeVec3(me, "pos");
    const double x = pos[0], y = pos[1], zf = pos[2];
    const bool grounded = me.value("grounded", true);
    const double energy = finiteOr(me.value("energy", 100.0), 100.0);
    const double maxEnergy = std::max(1.0, finiteOr(me.value("max_energy", 100.0), 100.0));

    if (grounded) {
        if (nav_.onRoofObst(x, y, zf)) {
            escRoute_.reset();
            escFlying_ = false;
            return false;
        }
        if (!nav_.offMain(x, y)) {
            escRoute_.reset();
            escFlying_ = false;
            return false;
        }
        escFlying_ = false;
        if (!escRoute_ || now_ - escAt_ > 6.0) {
            escRoute_ = nav_.escapePlan(x, y);
            escAt_ = now_;
            if (!escRoute_) return false;
        }
        if (std::hypot(escRoute_->launch[0] - x, escRoute_->launch[1] - y) > 2.5) {
            const Vec2 mv = travel(me, escRoute_->launch[0], escRoute_->launch[1]);
            mx = mv[0];
            my = mv[1];
            return true;
        }
        if (energy < std::min(70.0, maxEnergy * 0.7)) {
            mx = 0.0;
            my = 0.0;
            return true;
        }
        escFlying_ = true;
        jump = true;
    } else if (!escFlying_ || !escRoute_) {
        return false;
    }
    const double dx = escRoute_->land[0] - x, dy = escRoute_->land[1] - y;
    const double d = std::hypot(dx, dy);
    if (d < 1.5) return true;
    mx = dx / d;
    my = dy / d;
    double need = -1e9;
    const double lim = std::min(d, 30.0);
    for (double t = 0.0; t <= lim; t += 1.0) {
        need = std::max(need, arena_.topSurface(x + mx * t, y + my * t));
    }
    boost = (zf < need + 2.5) && energy > 1.0;
    return true;
}

bool Brain::dodgeShell(const Json& obs, const Json& me, double& mx, double& my) const {
    const auto pit = obs.find("projectiles");
    if (pit == obs.end() || !pit->is_array()) return false;
    const Vec3 pos = safeVec3(me, "pos");
    const auto& st = me.contains("stats") && me["stats"].is_object() ? me["stats"] : Json::object();
    const double rad = cfgMortarSplash_ + finiteOr(st.value("radius", cfgMechRadius_), cfgMechRadius_) + 2.0;

    for (const auto& pr : *pit) {
        if (!pr.is_object() || pr.value("kind", "") != std::string_view("mortar")) continue;
        Vec3 p = safeVec3(pr, "pos");
        const Vec3 v = safeVec3(pr, "vel");
        double vz = v[2];
        for (double t = 0.0; t < 4.0; t += 0.05) {
            p[0] += v[0] * 0.05;
            p[1] += v[1] * 0.05;
            p[2] += vz * 0.05;
            vz -= cfgMortarGravity_ * 0.05;
            if (arena_.isSolidPoint(p[0], p[1], p[2])) {
                double d = std::hypot(p[0] - pos[0], p[1] - pos[1]);
                if (d < rad && t < 2.5) {
                    double dx = pos[0] - p[0], dy = pos[1] - p[1];
                    if (d < 0.5) {
                        dx = -v[1];
                        dy = v[0];
                        d = std::hypot(dx, dy);
                    }
                    if (d > 1e-6) {
                        dx /= d;
                        dy /= d;
                        for (const double rot : {0.0, 0.7, -0.7, 1.4, -1.4}) {
                            const double c = std::cos(rot), s = std::sin(rot);
                            const double ex = dx * c - dy * s, ey = dx * s + dy * c;
                            if (nav_.lineWalkable(pos[0], pos[1], pos[0] + ex * 6.0, pos[1] + ey * 6.0)) {
                                mx = ex;
                                my = ey;
                                return true;
                            }
                        }
                    }
                }
                break;
            }
        }
    }
    return false;
}

Vec2 Brain::travel(const Json& me, double gx, double gy) {
    const Vec3 pos = safeVec3(me, "pos");
    const double x = pos[0], y = pos[1];
    const double d = std::hypot(gx - x, gy - y);
    if (d < 0.8) return {0.0, 0.0};

    auto unit = [&](double tx, double ty) -> Vec2 {
        const double l = std::hypot(tx - x, ty - y);
        if (l < 1e-6) return {0.0, 0.0};
        return {(tx - x) / l, (ty - y) / l};
    };

    if (nav_.onRoofObst(x, y, pos[2])) {
        return unit(gx, gy);
    }
    if (d < 45.0 && nav_.lineWalkable(x, y, gx, gy)) {
        return unit(gx, gy);
    }
    if (path_.empty() || std::hypot(gx - pathGoal_[0], gy - pathGoal_[1]) > 5.0 || now_ - pathAt_ > 2.5) {
        auto [p, reached] = nav_.path(x, y, gx, gy, 22000);
        (void)reached;
        path_ = std::move(p);
        pathGoal_ = {gx, gy};
        pathAt_ = now_;
        pathI_ = 0;
    }
    if (path_.empty()) {
        return unit(gx, gy);
    }

    std::size_t bi = pathI_;
    double bd = kInf;
    const std::size_t scanEnd = std::min(path_.size(), pathI_ + 10);
    for (std::size_t i = pathI_; i < scanEnd; ++i) {
        const Vec2 pp = nav_.cellPos(path_[i]);
        const double dd = std::hypot(pp[0] - x, pp[1] - y);
        if (dd < bd) {
            bi = i;
            bd = dd;
        }
    }
    pathI_ = bi;
    if (bd > 12.0) {
        path_.clear();
        return unit(gx, gy);
    }
    const std::size_t lookEnd = std::min(bi + 14, path_.size() - 1);
    for (std::size_t i = lookEnd; i > bi; --i) {
        const Vec2 pp = nav_.cellPos(path_[i]);
        if (nav_.lineWalkable(x, y, pp[0], pp[1])) {
            if (i == path_.size() - 1 && std::hypot(pp[0] - gx, pp[1] - gy) < 4.0 &&
                nav_.lineWalkable(x, y, gx, gy)) {
                return unit(gx, gy);
            }
            return unit(pp[0], pp[1]);
        }
    }
    if (bi + 1 < path_.size()) {
        const Vec2 pp = nav_.cellPos(path_[bi + 1]);
        return unit(pp[0], pp[1]);
    }
    return unit(gx, gy);
}

double Brain::prefRange(const Json& me, const std::array<std::optional<WeaponInfo>, 2>& ws, const MechTrack& t) const {
    const WeaponInfo* light = getWeapon(ws, "lightning");
    const WeaponInfo* laser = getWeapon(ws, "laser");
    double r = p_.gunRange;
    if (light && finiteOr(me.value("energy", 100.0), 100.0) > 12.0) {
        r = p_.lightRange;
    } else if (laser) {
        r = p_.laserRange;
    }
    if (t.kind == "npc") return 58.0;
    if (!light) {
        if (holdsKind(t.weapons, "lightning")) r = std::max(r, 64.0);
        if (holdsKind(t.weapons, "mortar")) r = std::max(r, 80.0);
    }
    return r;
}

Vec2 Brain::fightMove(const Json& obs, const Json& me, const std::array<std::optional<WeaponInfo>, 2>& ws,
                      const MechTrack& tg, const std::vector<MechTrack*>& seen, double wantLos) {
    const Vec3 pos = safeVec3(me, "pos");
    const double x = pos[0], y = pos[1];
    const auto& st = me.contains("stats") && me["stats"].is_object() ? me["stats"] : Json::object();
    const double mh = finiteOr(st.value("muzzle_height", cfgMechMuzzle_), cfgMechMuzzle_);
    const double ownH = finiteOr(st.value("height", cfgMechHeight_), cfgMechHeight_);
    const auto& z = obs.contains("zone") && obs["zone"].is_object() ? obs["zone"] : Json::object();

    const double rp = prefRange(me, ws, tg);
    const double bx = tg.pos[0] - x, by = tg.pos[1] - y;
    const double bd = std::hypot(bx, by) + 1e-9;
    const double ux = bx / bd, uy = by / bd;

    if (now_ >= strafeUntil_) {
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        if (u01(rng_) < 0.6) strafe_ = -strafe_;
        strafeUntil_ = now_ + 0.3 + u01(rng_) * 0.7;
    }
    const double sx = -uy * strafe_, sy = ux * strafe_;
    const Vec3 tc = centerOf(tg);

    const KnownItem* nearCube = nullptr;
    double nearCubeD = 40.0;
    for (const auto& [cid, c] : cubes_) {
        (void)cid;
        const double d = std::hypot(c.pos[0] - x, c.pos[1] - y);
        if (d < nearCubeD && std::abs(c.pos[2] - pos[2]) < 4.0) {
            nearCube = &c;
            nearCubeD = d;
        }
    }

    double best = -kInf;
    Vec2 bestMove{0.0, 0.0};
    bool blockedSide = false;
    const int rings = p_.peek ? 2 : 1;

    for (int k = -1; k < 16 * rings; ++k) {
        double cx = x, cy = y, dx = 0.0, dy = 0.0, r = 0.0;
        if (k >= 0) {
            const double a = static_cast<double>(k % 16) * (std::numbers::pi / 8.0);
            r = 6.0 * static_cast<double>(1 + k / 16);
            dx = std::cos(a);
            dy = std::sin(a);
            cx = x + dx * r;
            cy = y + dy * r;
            if (!nav_.lineWalkable(x, y, cx, cy) || !nav_.free(cx, cy)) {
                if (k < 16 && dx * sx + dy * sy > 0.9) blockedSide = true;
                continue;
            }
        }
        const double cz = nav_.surfAt(cx, cy);
        const double d = std::hypot(tg.pos[0] - cx, tg.pos[1] - cy);
        double s = -1.1 * std::abs(d - rp) - r * 0.4;
        const bool los = !arena_.blocksLine({cx, cy, cz + mh}, tc);
        if (los) {
            s += 45.0 * wantLos;
        }
        if (p_.peek && k >= 0 && k < 16 && (los == (wantLos > 0.0))) {
            for (int q = 0; q < 16; q += 2) {
                const double qa = q * (std::numbers::pi / 8.0);
                const double qx = cx + std::cos(qa) * 7.0, qy = cy + std::sin(qa) * 7.0;
                if (nav_.free(qx, qy) &&
                    (!arena_.blocksLine({qx, qy, nav_.surfAt(qx, qy) + mh}, tc)) != los) {
                    s += 8.0;
                    break;
                }
            }
        }
        const Vec3 body{cx, cy, cz + ownH * 0.6};
        for (const MechTrack* e : seen) {
            if (e->id == tg.id || std::hypot(e->pos[0] - cx, e->pos[1] - cy) > 170.0) continue;
            if (!arena_.blocksLine(eyeOf(*e), body)) {
                s -= 30.0;
            }
        }
        s -= zonePenalty(z, cx, cy);
        if (nearCube) {
            s += 1.5 * (nearCubeD - std::hypot(nearCube->pos[0] - cx, nearCube->pos[1] - cy));
        }
        const double cl = nav_.clearanceAt(cx, cy);
        if (cl < 4.0) {
            s -= (4.0 - cl) * 2.5;
        }
        if (wantLos > 0.0) {
            s += 9.0 * (dx * sx + dy * sy);
        }
        if (k < 0) {
            s -= 8.0;
            if (wantLos < 0.0 && !los) s += 14.0;
        }
        if (s > best) {
            best = s;
            bestMove = {dx, dy};
        }
    }
    if (blockedSide) {
        strafe_ = -strafe_;
        strafeUntil_ = now_ + 0.5;
    }
    return bestMove;
}

bool Brain::coverMove(const Json& obs, const Json& me, const std::vector<MechTrack*>& seen, double& mx, double& my) {
    const Vec3 pos = safeVec3(me, "pos");
    const double x = pos[0], y = pos[1];
    const auto& st = me.contains("stats") && me["stats"].is_object() ? me["stats"] : Json::object();
    const double ownH = finiteOr(st.value("height", cfgMechHeight_), cfgMechHeight_);
    const auto& z = obs.contains("zone") && obs["zone"].is_object() ? obs["zone"] : Json::object();

    std::vector<MechTrack*> threats;
    for (MechTrack* t : seen) {
        if (std::hypot(t->pos[0] - x, t->pos[1] - y) < 230.0) {
            threats.push_back(t);
        }
    }
    if (threats.empty()) return false;
    if (threats.size() > 4) {
        std::sort(threats.begin(), threats.end(), [&](const MechTrack* a, const MechTrack* b) {
            return std::hypot(a->pos[0] - x, a->pos[1] - y) < std::hypot(b->pos[0] - x, b->pos[1] - y);
        });
        threats.resize(4);
    }

    auto hidden = [&](double cx, double cy) -> bool {
        const double cz = nav_.surfAt(cx, cy);
        for (const MechTrack* t : threats) {
            const Vec3 e = eyeOf(*t);
            if (!arena_.blocksLine(e, {cx, cy, cz + ownH}) ||
                !arena_.blocksLine(e, {cx, cy, cz + ownH * 0.5})) {
                return false;
            }
        }
        return true;
    };

    if (coverOK_ && now_ - coverAt_ < 0.4) {
        if (std::hypot(coverGoal_[0] - x, coverGoal_[1] - y) < 1.5) {
            mx = 0.0;
            my = 0.0;
        } else {
            const Vec2 mv = travel(me, coverGoal_[0], coverGoal_[1]);
            mx = mv[0];
            my = mv[1];
        }
        return true;
    }
    coverAt_ = now_;
    if (hidden(x, y) && zonePenalty(z, x, y) == 0.0) {
        coverOK_ = true;
        coverGoal_ = {x, y};
        mx = 0.0;
        my = 0.0;
        return true;
    }

    std::vector<Vec2> threatXY;
    threatXY.reserve(threats.size());
    for (const MechTrack* t : threats) threatXY.push_back({t->pos[0], t->pos[1]});

    const auto goal = nav_.findCoverBFS(
        x, y, hidden, [&](double cx, double cy) { return zonePenalty(z, cx, cy); }, threatXY, 50, 350);
    if (!goal) {
        coverOK_ = false;
        return false;
    }
    coverOK_ = true;
    coverGoal_ = *goal;
    if (std::hypot(coverGoal_[0] - x, coverGoal_[1] - y) < 1.5) {
        mx = 0.0;
        my = 0.0;
    } else {
        const Vec2 mv = travel(me, coverGoal_[0], coverGoal_[1]);
        mx = mv[0];
        my = mv[1];
    }
    return true;
}

bool Brain::hideFromBearing(const Json& obs, const Json& me, const std::vector<MechTrack*>& seen,
                            double& mx, double& my) {
    const Vec3 pos = safeVec3(me, "pos");
    MechTrack ghost;
    ghost.id = -999;
    ghost.kind = "mech";
    ghost.pos = {pos[0] + std::cos(hurtBearing_) * 80.0, pos[1] + std::sin(hurtBearing_) * 80.0, pos[2]};
    std::vector<MechTrack*> ext;
    ext.reserve(seen.size() + 1);
    ext.push_back(&ghost);
    ext.insert(ext.end(), seen.begin(), seen.end());
    return coverMove(obs, me, ext, mx, my);
}

std::optional<Brain::LootChoice> Brain::chooseLoot(const Json& obs, const Json& me,
                                                   const std::array<std::optional<WeaponInfo>, 2>& ws,
                                                   double hpFrac, bool danger) {
    const Vec3 pos = safeVec3(me, "pos");
    const double x = pos[0], y = pos[1];
    const auto& z = obs.contains("zone") && obs["zone"].is_object() ? obs["zone"] : Json::object();
    const auto& st = me.contains("stats") && me["stats"].is_object() ? me["stats"] : Json::object();
    const double walkSpeed = std::max(1.0, finiteOr(st.value("walk_speed", 11.7), 11.7));
    const int myCubes = static_cast<int>(finiteOr(me.value("cubes", 0.0), 0.0));
    const ZoneCircle cur = curZone(z);

    std::optional<LootChoice> best;
    double bestU = 0.0;

    auto consider = [&](const std::string& kind, int id, const Vec3& ipos, double value) {
        if (auto bi = badLoot_.find(id); bi != badLoot_.end() && bi->second > now_) return;
        const double d = std::hypot(ipos[0] - x, ipos[1] - y);
        if (d > 260.0) return;
        if (std::hypot(ipos[0] - cur.x, ipos[1] - cur.y) > cur.r - 6.0) return;
        const double dl = zoneDeadline(z, ipos[0], ipos[1], 6.0);
        if (now_ + d * 1.4 / walkSpeed + 5.0 > dl) return;
        if (ipos[2] - arena_.ground(ipos[0], ipos[1]) > 3.2) return;
        if (!nav_.isMainReachable(ipos[0], ipos[1])) return;
        double u = value / (d + 18.0);
        if (id == lootID_) u *= 1.35;
        if (u > bestU) {
            bestU = u;
            best = LootChoice{kind, id, ipos, value};
        }
    };

    // 1. Ground weapon pickups (240m laser seek!)
    for (const auto& [pid, p] : pickups_) {
        (void)pid;
        double rad = p_.lootRadius;
        const bool seek = (p.kind == "laser" && p_.laserSeek > rad);
        if (seek) rad = p_.laserSeek;
        if (wantPickup(ws, p.kind) && std::hypot(p.pos[0] - x, p.pos[1] - y) < rad) {
            double v = ws[1].has_value() ? 60.0 : 130.0;
            if (seek) v = std::max(v, 150.0);
            consider("pickup", p.id, p.pos, v);
        }
    }

    // 2. Power cubes
    const double cubeV = (myCubes >= cfgCubeStatCap_) ? 25.0 : 75.0;
    const double cubeRad = std::max(60.0, p_.lootRadius * 0.7);
    for (const auto& [cid, c] : cubes_) {
        (void)cid;
        if (std::hypot(c.pos[0] - x, c.pos[1] - y) < cubeRad) {
            consider("cube", c.id, c.pos, cubeV);
        }
    }

    // 3. Crates & NPC drones
    if (hpFrac > p_.farmHP && !danger && now_ < p_.crateUntil) {
        for (const auto& [cid, c] : crates_) {
            (void)cid;
            if (std::hypot(c.pos[0] - x, c.pos[1] - y) < p_.lootRadius) {
                consider("crate", c.id, c.pos, cubeV * 0.6);
            }
        }
        if (p_.farmNPC && myCubes < cfgCubeStatCap_) {
            for (const auto& [nid, n] : npcs_) {
                (void)nid;
                if (now_ - n.seen < 12.0 && std::hypot(n.pos[0] - x, n.pos[1] - y) < p_.lootRadius) {
                    consider("npc", n.id, n.pos, 85.0);
                }
            }
        }
    }

    if (!best) {
        lootID_ = -1;
        return std::nullopt;
    }
    if (best->id != lootID_) {
        lootID_ = best->id;
        lootSince_ = now_;
    } else if (now_ - lootSince_ > 30.0) {
        badLoot_[best->id] = now_ + 60.0;
        lootID_ = -1;
        return std::nullopt;
    }
    return best;
}

Vec2 Brain::exploreMove(const Json& obs, const Json& me) {
    const Vec3 pos = safeVec3(me, "pos");
    const auto& z = obs.contains("zone") && obs["zone"].is_object() ? obs["zone"] : Json::object();
    const Vec2 nc = safeVec2(z, "next_center");
    const double nr = finiteOr(z.value("next_radius", 200.0), 200.0);

    if (now_ > exploreUntil_ || std::hypot(exploreXY_[0] - pos[0], exploreXY_[1] - pos[1]) < 6.0) {
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        for (int tries = 0; tries < 20; ++tries) {
            const double a = u01(rng_) * kTau;
            const double r = std::sqrt(u01(rng_)) * nr * 0.8;
            const double px = nc[0] + std::cos(a) * r, py = nc[1] + std::sin(a) * r;
            if (!nav_.inBounds(px, py) || !nav_.free(px, py)) continue;
            const double d = std::hypot(px - pos[0], py - pos[1]);
            if (d < 50.0 || d > 220.0) continue;
            exploreXY_ = {px, py};
            exploreUntil_ = now_ + 30.0;
            break;
        }
    }
    if (now_ > exploreUntil_) return {0.0, 0.0};
    return travel(me, exploreXY_[0], exploreXY_[1]);
}

double Brain::homeCost(const Json& obs, const Json& me, double px, double py) const {
    const Vec3 pos = safeVec3(me, "pos");
    const auto& z = obs.contains("zone") && obs["zone"].is_object() ? obs["zone"] : Json::object();
    const Vec2 nc = safeVec2(z, "next_center");
    const double nr = finiteOr(z.value("next_radius", 200.0), 200.0);
    double crowd = 0.0;
    for (const auto& [id, t] : tracks_) {
        (void)id;
        if (now_ - t.seen < 20.0) {
            crowd += std::max(0.0, 80.0 - std::hypot(t.pos[0] - px, t.pos[1] - py)) / 80.0;
        }
    }
    if (const auto dit = obs.find("drops"); dit != obs.end() && dit->is_array()) {
        for (const auto& dr : *dit) {
            if (!dr.is_object()) continue;
            const Vec3 dp = safeVec3(dr, "pos");
            crowd += std::max(0.0, 60.0 - std::hypot(dp[0] - px, dp[1] - py)) / 60.0;
        }
    }
    const double edge = std::hypot(px - nc[0], py - nc[1]) / std::max(30.0, nr);
    return p_.homeExpo * nav_.exposure(px, py) + 0.25 * std::hypot(px - pos[0], py - pos[1]) + 40.0 * crowd + 10.0 * edge;
}

std::optional<Vec2> Brain::findHome(const Json& obs, const Json& me) {
    const auto& z = obs.contains("zone") && obs["zone"].is_object() ? obs["zone"] : Json::object();
    const Vec2 nc = safeVec2(z, "next_center");
    const double nr = finiteOr(z.value("next_radius", 200.0), 200.0);
    const bool out = !homeSet_ || std::hypot(homeXY_[0] - nc[0], homeXY_[1] - nc[1]) > nr * 0.8;
    if (out || now_ - homeAt_ > 12.0) {
        homeAt_ = now_;
        Vec2 best{};
        double bestC = kInf;
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        for (int k = 0; k < 40; ++k) {
            const double a = u01(rng_) * kTau;
            const double r = std::sqrt(u01(rng_)) * nr * 0.7;
            const double px = nc[0] + std::cos(a) * r, py = nc[1] + std::sin(a) * r;
            if (!nav_.inBounds(px, py) || !nav_.reachable(px, py)) continue;
            const double c = homeCost(obs, me, px, py);
            if (c < bestC) {
                bestC = c;
                best = {px, py};
            }
        }
        if (std::isfinite(bestC) && (out || bestC < homeCost(obs, me, homeXY_[0], homeXY_[1]) - 15.0)) {
            homeXY_ = best;
            homeSet_ = true;
        }
    }
    if (!homeSet_) return std::nullopt;
    return homeXY_;
}

void Brain::dodgeBullets(const Json& obs, const Json& me, double& dx, double& dy, bool& enemyLaser) const {
    dx = 0.0;
    dy = 0.0;
    enemyLaser = false;
    const Vec3 pos = safeVec3(me, "pos");
    const auto& st = me.contains("stats") && me["stats"].is_object() ? me["stats"] : Json::object();
    const double cx = pos[0], cy = pos[1];
    const double cz = pos[2] + finiteOr(st.value("height", cfgMechHeight_), cfgMechHeight_) * 0.56;
    const double reach = finiteOr(st.value("radius", cfgMechRadius_), cfgMechRadius_) + 1.1;

    if (const auto pit = obs.find("projectiles"); pit != obs.end() && pit->is_array()) {
        for (const auto& pr : *pit) {
            if (!pr.is_object() || pr.value("kind", "") == std::string_view("mortar")) continue;
            const Vec3 p = safeVec3(pr, "pos");
            const Vec3 v = safeVec3(pr, "vel");
            const double v2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            if (v2 < 1.0) continue;
            const double tc = ((cx - p[0]) * v[0] + (cy - p[1]) * v[1] + (cz - p[2]) * v[2]) / v2;
            if (tc <= 0.0 || tc > 0.7) continue;
            const double mxx = p[0] + v[0] * tc - cx, myy = p[1] + v[1] * tc - cy;
            if (std::hypot(mxx, myy) < reach) {
                const double vh = std::hypot(v[0], v[1]);
                if (vh < 1.0) continue;
                double px = -v[1] / vh, py = v[0] / vh;
                if (px * mxx + py * myy > 0.0) {
                    px = -px;
                    py = -py;
                }
                dx += px;
                dy += py;
            }
        }
    }
    if (const auto mit = obs.find("mechs"); mit != obs.end() && mit->is_array()) {
        for (const auto& m : *mit) {
            if (!m.is_object() || m.value("kind", "") != std::string_view("mech") || !m.value("charging", false)) continue;
            const Vec3 mp = safeVec3(m, "pos");
            const Vec2 maim = safeVec2(m, "aim");
            const double toMe = std::atan2(cy - mp[1], cx - mp[0]);
            if (std::abs(angDiff(maim[0], toMe)) > 0.15) continue;
            dx += std::cos(toMe + std::numbers::pi * 0.5 * strafe_) * 1.5;
            dy += std::sin(toMe + std::numbers::pi * 0.5 * strafe_) * 1.5;
            enemyLaser = true;
        }
    }
    const double l = std::hypot(dx, dy);
    if (l > 0.01) {
        dx /= l;
        dy /= l;
    } else {
        dx = 0.0;
        dy = 0.0;
        enemyLaser = false;
    }
}

Vec2 Brain::leadAim(const Vec3& muzzle, const Vec3& tp, const Vec3& tv, double speed, bool grounded) const {
    if (speed <= 0.0) speed = 252.0;
    double t = 0.0;
    double px = tp[0], py = tp[1], pz = tp[2];
    for (int k = 0; k < 4; ++k) {
        const double tt = kLat + t;
        px = tp[0] + tv[0] * tt;
        py = tp[1] + tv[1] * tt;
        pz = tp[2] + (grounded ? 0.0 : tv[2] * tt * 0.5);
        const double d = std::sqrt((px - muzzle[0]) * (px - muzzle[0]) +
                                   (py - muzzle[1]) * (py - muzzle[1]) +
                                   (pz - muzzle[2]) * (pz - muzzle[2]));
        t = std::max(0.0, d - 3.0) / speed;
    }
    return {std::atan2(py - muzzle[1], px - muzzle[0]),
            std::atan2(pz - muzzle[2], std::hypot(px - muzzle[0], py - muzzle[1]))};
}

std::optional<Vec2> Brain::mortarAim(const Vec3& tube, const Vec3& foot, const Vec3& tv,
                                     double v, double g, double minPitch, double maxPitch) const {
    if (v <= 0.0 || g <= 0.0) return std::nullopt;
    double t = 0.0, yaw = 0.0, pitch = 0.0;
    for (int k = 0; k < 3; ++k) {
        const double px = foot[0] + tv[0] * (kLat + t);
        const double py = foot[1] + tv[1] * (kLat + t);
        const double dx = std::hypot(px - tube[0], py - tube[1]);
        const double dz = foot[2] - tube[2];
        const double disc = v * v * v * v - g * (g * dx * dx + 2.0 * dz * v * v);
        if (disc < 0.0 || dx < 1.0) return std::nullopt;
        pitch = std::atan((v * v - std::sqrt(disc)) / (g * dx));
        yaw = std::atan2(py - tube[1], px - tube[0]);
        t = dx / (v * std::cos(pitch));
    }
    if (pitch < minPitch || pitch > maxPitch) return std::nullopt;
    const double c = std::cos(pitch), s = std::sin(pitch);
    for (double tt = 0.1; tt < t * 0.9; tt += 0.1) {
        const double d = v * c * tt;
        const double px = tube[0] + std::cos(yaw) * d;
        const double py = tube[1] + std::sin(yaw) * d;
        const double pz = tube[2] + v * s * tt - 0.5 * g * tt * tt;
        if (arena_.isSolidPoint(px, py, pz)) return std::nullopt;
    }
    return Vec2{yaw, pitch};
}

Json Brain::act(const Json& obs) {
    const auto& me = obs.contains("self") && obs["self"].is_object() ? obs["self"] : Json::object();
    const auto& z = obs.contains("zone") && obs["zone"].is_object() ? obs["zone"] : Json::object();
    const auto& st = me.contains("stats") && me["stats"].is_object() ? me["stats"] : Json::object();
    now_ = finiteOr(obs.value("t", obs.value("tick", 0.0) / 20.0), 0.0);

    const auto ws = parseWeapons(me);
    observe(obs, me, ws);

    const Vec3 pos = safeVec3(me, "pos");
    const Vec3 vel = safeVec3(me, "vel");
    const Vec2 curAim = safeVec2(me, "aim");
    const double x = pos[0], y = pos[1], zf = pos[2];
    const double hp = finiteOr(me.value("hp", 1400.0), 1400.0);
    const double maxHp = std::max(1.0, finiteOr(me.value("max_hp", 1400.0), 1400.0));
    const double hpFrac = hp / maxHp;
    const double energy = finiteOr(me.value("energy", 100.0), 100.0);
    const bool grounded = me.value("grounded", true);
    const bool inZone = me.value("in_zone", true);
    const int myCubes = static_cast<int>(finiteOr(me.value("cubes", 0.0), 0.0));
    const int alive = obs.value("alive", 16);

    const double walkSpeed = std::max(1.0, finiteOr(st.value("walk_speed", 11.7), 11.7));
    const double turnRate = finiteOr(st.value("turn_rate", 4.0), 4.0);
    const double pitchRate = finiteOr(st.value("pitch_rate", 3.0), 3.0);
    const double minPitch = finiteOr(st.value("min_pitch", -0.7), -0.7);
    const double maxPitch = finiteOr(st.value("max_pitch", 1.3), 1.3);
    const double ownMuzzleH = finiteOr(st.value("muzzle_height", cfgMechMuzzle_), cfgMechMuzzle_);
    const double ownHeight = finiteOr(st.value("height", cfgMechHeight_), cfgMechHeight_);

    const WeaponInfo* gun = getWeapon(ws, "gun");
    const WeaponInfo* light = getWeapon(ws, "lightning");
    const WeaponInfo* laser = getWeapon(ws, "laser");

    const double nextRadius = finiteOr(z.value("next_radius", 380.0), 380.0);
    const bool late = nextRadius <= 70.0 || alive <= 3;

    // 1. Target selection & engagement classification
    std::vector<MechTrack*> seen;
    seen.reserve(tracks_.size());
    MechTrack* target = nullptr;
    MechTrack* hidden = nullptr;
    double bestScore = kInf;

    for (auto& [id, t] : tracks_) {
        (void)id;
        if (now_ - t.seen > 2.5) continue;
        t.engage = false;
        seen.push_back(&t);
        if (t.seen != now_) {
            t.los = false;
            continue;
        }
        const double toMe = std::atan2(y - t.pos[1], x - t.pos[0]);
        const double off = std::abs(angDiff(t.aim[0], toMe));
        const bool aimed = off < 0.22;
        if (p_.ghost) {
            t.engage = t.dist < p_.engageDist || now_ - t.hitMe < 4.0 ||
                (aimed && t.charging && t.dist < 120.0) ||
                (t.hp < 350.0 && t.dist < 120.0) || alive <= 2;
        } else {
            t.engage = t.dist < p_.engageDist || now_ - t.hitMe < 6.0 || late ||
                (t.closing > 3.0 && t.dist < 180.0) ||
                (t.hp < 0.4 * t.maxHp && t.dist < 160.0) ||
                (aimed && t.dist < 190.0);
        }
        if (t.engage && !t.los && t.dist < p_.maxTarget && (!hidden || t.dist < hidden->dist)) {
            hidden = &t;
        }
        if (!t.engage || !t.los || t.dist > p_.maxTarget) continue;

        double s = t.dist + p_.hpW * t.hp;
        if (p_.lowBonus > 0.0 && t.hp <= 0.30 * t.maxHp) s -= p_.lowBonus;
        if (p_.stick > 0.0 && t.id == stickID_ && now_ - stickAt_ < 2.0) s -= p_.stick;
        if (now_ - t.hitMe < 4.0) s -= 40.0;
        if (t.closing > 3.0) s -= 15.0;
        if (s < bestScore) {
            bestScore = s;
            target = &t;
        }
    }
    if (target) {
        stickID_ = target->id;
        stickAt_ = now_;
    }
    MechTrack* foe = target;
    if (!foe && p_.peek) foe = hidden;

    bool danger = false;
    int crowd = 0;
    for (const MechTrack* t : seen) {
        if (t->engage || now_ - t->hitMe < 4.0) danger = true;
        if (t->seen == now_ && (t->engage || (t->los && t->dist < 150.0))) ++crowd;
    }
    if (danger || now_ - lastHurt_ < 3.0) {
        lastThreatAt_ = now_;
    }

    // 2. Disciplined Retreat / LoseRule
    const bool finishing = target && target->dist < 120.0 && (target->hp < 200.0 || target->hp < hp * 0.5);
    const bool outgunned = target && holdsKind(target->weapons, "laser") && !laser && target->dist > 55.0 && hpFrac < 0.8;
    const bool swamped = p_.crowdHide && (crowd >= 3 || (crowd == 2 && hpFrac < 0.65) || outgunned);
    if (!retreat_ && ((hpFrac < p_.retreatHP && now_ - lastThreatAt_ < 2.0) || swamped)) {
        retreat_ = true;
        coverAt_ = -1.0;
    }
    if (retreat_ && !swamped && (hpFrac > p_.recoverHP || (now_ - lastThreatAt_ > 7.0 && hpFrac > 0.55))) {
        retreat_ = false;
    }
    if (p_.loseRule) {
        int near = 0;
        for (const MechTrack* t : seen) {
            if (t->seen == now_ && t->dist < 160.0) ++near;
        }
        bool losing = (near >= 2 && hpFrac < 0.55) || (near >= 3 && hpFrac < 0.80);
        if (target) {
            const double tf = target->hp / std::max(1.0, target->maxHp);
            if (hpFrac < 0.35 && tf > hpFrac + 0.10) losing = true;
            if (tf < 0.25 && hpFrac > 0.20 && near <= 2) losing = false;
        }
        if (losing) loseAt_ = now_;
        retreat_ = (loseAt_ > 0.0 && now_ - loseAt_ < 1.5);
    }

    // 3. Mutually-Exclusive Movement Priority Cascade (ZERO rule conflicts!)
    double mx = 0.0, my = 0.0;
    bool boost = false;
    bool jump = false;
    std::string mode = "idle";
    MechTrack* farmNpc = nullptr;
    std::optional<Vec3> farmCratePos;

    const Vec2 zg = zoneGoal(z, x, y);
    const double zdist = std::hypot(zg[0] - x, zg[1] - y);
    const double tNeed = zdist * 1.45 / walkSpeed + 4.0;
    const double deadline = zoneDeadline(z, x, y, 8.0);
    const bool zoneMust = !inZone || now_ + tNeed + 3.0 >= deadline;
    const bool zoneShould = now_ + tNeed + 14.0 >= deadline;

    if (runEscape(me, mx, my, boost, jump)) {
        mode = "escape";
    } else if (now_ < unstuckUntil_) {
        mx = unstuckDir_[0];
        my = unstuckDir_[1];
        mode = "unstuck";
    } else if (dodgeShell(obs, me, mx, my)) {
        boost = energy > 15.0;
        mode = "dodge-mortar";
    } else if (zoneMust && zdist > 2.0) {
        const Vec2 mv = travel(me, zg[0], zg[1]);
        mx = mv[0];
        my = mv[1];
        boost = (!inZone || now_ + tNeed >= deadline) && energy > 5.0;
        mode = "zone";
    } else if (retreat_ && !finishing && !late) {
        if (coverMove(obs, me, seen, mx, my)) {
            boost = energy > 10.0 && (mx != 0.0 || my != 0.0);
            mode = "cover";
        } else if (target) {
            const Vec2 mv = fightMove(obs, me, ws, *target, seen, 1.0);
            mx = mv[0];
            my = mv[1];
            mode = "cornered";
        } else {
            const Vec2 mv = travel(me, zg[0], zg[1]);
            mx = mv[0];
            my = mv[1];
            if (npcHurt_ > 0.0 && now_ - npcHurt_ < 3.0 && mx == 0.0 && my == 0.0) {
                const double side = std::fmod(now_, 3.0) > 1.5 ? -1.0 : 1.0;
                mx = -std::sin(npcBearing_) * side;
                my = std::cos(npcBearing_) * side;
            }
            mode = "flee";
        }
    } else if (!target && now_ - lastHurt_ < 2.5 && !late && p_.ghost && hideFromBearing(obs, me, seen, mx, my)) {
        boost = energy > 30.0 && (mx != 0.0 || my != 0.0);
        mode = "cover";
    } else if (foe) {
        bool ready = gun && gun->ammo > 0.0 && gun->reload <= 0.25 && foe->dist <= gun->range - 10.0;
        const bool hot = laser && laser->charge >= laser->fullCharge - 0.3 && foe->dist <= laser->range - 5.0;
        if (hot || (light && foe->dist <= light->range + 6.0 && energy >= light->energyCost * 3.0)) {
            ready = true;
        }
        if (!p_.peek && (light || laser)) {
            ready = true;
        }
        double wantLos = ready ? 1.0 : -1.0;
        if (p_.peek && !hot) {
            for (const MechTrack* t : seen) {
                if (t->seen == now_ && t->charging &&
                    std::abs(angDiff(t->aim[0], std::atan2(y - t->pos[1], x - t->pos[0]))) < 0.3) {
                    wantLos = -1.6;
                }
            }
        }
        if (p_.finish > 0.0 && foe->hp < p_.finishHP * foe->maxHp && hpFrac > 0.42 && crowd <= 2) {
            const BrainParams saved = p_;
            p_.gunRange = p_.finish;
            p_.laserRange = p_.finish;
            p_.lightRange = p_.finish;
            const Vec2 mv = fightMove(obs, me, ws, *foe, seen, wantLos);
            p_ = saved;
            mx = mv[0];
            my = mv[1];
        } else {
            const Vec2 mv = fightMove(obs, me, ws, *foe, seen, wantLos);
            mx = mv[0];
            my = mv[1];
        }
        if (light && foe->dist > 50.0 && energy > 55.0 && wantLos > 0.0) {
            boost = true;
        }
        mode = "fight";
        if (p_.fightLoot > 0.0) {
            double r = p_.fightLoot * (hpFrac <= 0.5 ? 0.57 : 1.0);
            std::optional<Vec3> bc;
            for (const auto& [cid, c] : cubes_) {
                (void)cid;
                const double d = std::hypot(c.pos[0] - x, c.pos[1] - y);
                if (d < r && std::abs(c.pos[2] - zf) < 4.0 &&
                    zonePenalty(z, c.pos[0], c.pos[1]) == 0.0 &&
                    nav_.isMainReachable(c.pos[0], c.pos[1])) {
                    r = d;
                    bc = c.pos;
                }
            }
            if (bc) {
                const Vec2 mv = travel(me, (*bc)[0], (*bc)[1]);
                mx = mv[0];
                my = mv[1];
                mode = "fight:cube";
            }
        }
    } else if (zoneShould && zdist > 2.0) {
        const Vec2 mv = travel(me, zg[0], zg[1]);
        mx = mv[0];
        my = mv[1];
        mode = "zone";
    } else {
        if (const auto lc = chooseLoot(obs, me, ws, hpFrac, danger)) {
            mode = "loot:" + lc->kind;
            if (lc->kind == "npc") {
                auto nit = npcs_.find(lc->id);
                if (nit != npcs_.end() && now_ - nit->second.seen < 1.0 && nit->second.los && nit->second.dist < 75.0) {
                    farmNpc = &nit->second;
                    const Vec2 mv = fightMove(obs, me, ws, nit->second, seen, 1.0);
                    mx = mv[0];
                    my = mv[1];
                } else {
                    const Vec2 mv = travel(me, lc->pos[0], lc->pos[1]);
                    mx = mv[0];
                    my = mv[1];
                }
            } else if (lc->kind == "crate") {
                const double d = std::hypot(lc->pos[0] - x, lc->pos[1] - y);
                const Vec3 cp{lc->pos[0], lc->pos[1], lc->pos[2] + cfgCrateHeight_ * 0.5};
                if (d < 50.0 && !arena_.blocksLine({x, y, zf + ownMuzzleH}, cp)) {
                    farmCratePos = cp;
                    if (d < 12.0) {
                        mx = (x - lc->pos[0]) / std::max(d, 0.1);
                        my = (y - lc->pos[1]) / std::max(d, 0.1);
                    }
                } else {
                    const Vec2 mv = travel(me, lc->pos[0], lc->pos[1]);
                    mx = mv[0];
                    my = mv[1];
                }
            } else {
                const Vec2 mv = travel(me, lc->pos[0], lc->pos[1]);
                mx = mv[0];
                my = mv[1];
                if (lc->kind == "drop" && std::hypot(lc->pos[0] - x, lc->pos[1] - y) < 2.5) {
                    mx = 0.0;
                    my = 0.0;
                    mode = "cap-drop";
                }
            }
        } else if (zdist > 3.0 && (!p_.ghost || nextRadius <= 30.0)) {
            const Vec2 mv = travel(me, zg[0], zg[1]);
            mx = mv[0];
            my = mv[1];
            mode = "zone-walk";
        } else if (p_.ghost) {
            if (const auto hm = findHome(obs, me); hm && std::hypot((*hm)[0] - x, (*hm)[1] - y) > 3.0) {
                const Vec2 mv = travel(me, (*hm)[0], (*hm)[1]);
                mx = mv[0];
                my = mv[1];
                if (!danger && now_ - lastHurt_ > 6.0) {
                    const double q = std::min(1.0, 4.6 / walkSpeed);
                    mx *= q;
                    my *= q;
                }
                mode = "home";
            }
        } else if (p_.explore && !late && myCubes < cfgCubeStatCap_ && hpFrac > 0.70 && !danger) {
            const Vec2 mv = exploreMove(obs, me);
            mx = mv[0];
            my = mv[1];
            mode = "explore";
        }
    }

    // 4. Additive 3D Incoming Bullet & Charging-Laser Dodge Overlay
    if (p_.dodge && mode != "unstuck" && mode != "escape") {
        double ddx = 0.0, ddy = 0.0;
        bool lz = false;
        dodgeBullets(obs, me, ddx, ddy, lz);
        if (ddx != 0.0 || ddy != 0.0) {
            mx = mx * 0.4 + ddx * 0.9;
            my = my * 0.4 + ddy * 0.9;
            const double l = std::hypot(mx, my);
            if (l > 1.0) {
                mx /= l;
                my /= l;
            }
            if (lz && energy > 25.0) boost = true;
        }
    }

    // 5. Fast Physical Unstuck Detector
    if (mode == "escape") {
        stuckSince_ = -1.0;
    } else if ((mx != 0.0 || my != 0.0) && std::hypot(vel[0], vel[1]) < 1.2 && grounded) {
        if (stuckSince_ < 0.0) {
            stuckSince_ = now_;
        } else if (now_ - stuckSince_ > 0.7) {
            std::uniform_real_distribution<double> u01(0.0, 1.0);
            const double a = u01(rng_) * kTau;
            unstuckDir_ = {std::cos(a), std::sin(a)};
            unstuckUntil_ = now_ + 0.5;
            stuckSince_ = -1.0;
            path_.clear();
            jump = true;
        }
    } else {
        stuckSince_ = -1.0;
    }

    // 6. Combat Aiming, Half-Tick Slew Prediction, Laser Mastery & Firing
    const Vec3 muzzle{x + vel[0] * kLat, y + vel[1] * kLat, zf + ownMuzzleH};
    Vec2 aim{curAim[0], 0.0};
    bool haveAim = false;
    Vec3 tp{}, tv{};
    double tr = 2.5, th = 9.0;
    bool shootable = false;
    bool isMech = false;
    bool defendNpc = false;

    if (!target && npcHurt_ > 0.0 && now_ - npcHurt_ < 3.0) {
        MechTrack* worst = nullptr;
        double bestOff = 0.7;
        for (auto& [nid, n] : npcs_) {
            (void)nid;
            if (now_ - n.seen > 0.5 || !n.los || n.dist > 150.0) continue;
            double off = std::abs(angDiff(std::atan2(n.pos[1] - y, n.pos[0] - x), npcBearing_));
            if (now_ - n.hitMe < 3.0) off = 0.0;
            if (off < bestOff) {
                bestOff = off;
                worst = &n;
            }
        }
        if (worst) {
            farmNpc = worst;
            farmCratePos.reset();
            defendNpc = true;
        }
    }

    if (target) {
        const double sc = scaleOf(target->cubes);
        tr = cfgMechRadius_ * sc;
        th = cfgMechHeight_ * sc;
        tp = target->pos;
        tv = target->vel;
        tp[2] += th * 0.5;
        if (arena_.blocksLine(muzzle, tp)) {
            tp[2] = target->pos[2] + th * 0.85;
        }
        shootable = true;
        isMech = true;
    } else if (farmNpc) {
        tr = cfgNpcRadius_;
        th = cfgNpcHeight_;
        tp = farmNpc->pos;
        tv = farmNpc->vel;
        tp[2] += th * 0.5;
        shootable = true;
    } else if (farmCratePos) {
        tr = cfgCrateRadius_;
        th = cfgCrateHeight_;
        tp = *farmCratePos;
        tv = {0.0, 0.0, 0.0};
        shootable = true;
    }
    if (!isMech && !target && (retreat_ || hpFrac < 0.5) && !defendNpc) {
        shootable = false;
    }

    std::array<bool, 2> fire{false, false};
    if (shootable) {
        const double dist = std::hypot(tp[0] - x, tp[1] - y);
        const bool targetGrounded = std::abs(tv[2]) < 0.5;
        Vec2 gAim{0.0, 0.0};
        bool gunOK = false;
        if (gun) {
            gAim = leadAim(muzzle, tp, tv, gun->speed, targetGrounded);
            gunOK = dist <= gun->range * 0.96;
        }
        const Vec3 dp{tp[0] + tv[0] * (kLat + 0.05), tp[1] + tv[1] * (kLat + 0.05), tp[2]};
        const double hy = std::atan2(dp[1] - muzzle[1], dp[0] - muzzle[0]);
        const double hpPitch = std::atan2(dp[2] - muzzle[2], std::hypot(dp[0] - muzzle[0], dp[1] - muzzle[1]));

        aim = gun ? gAim : Vec2{hy, hpPitch};
        haveAim = true;

        const double tolY = std::atan2(tr * 0.75, std::max(dist, 1.0));
        const double tolP = std::atan2(th * 0.40, std::max(dist, 1.0));
        auto slewAligned = [&](double wantY, double wantP) -> bool {
            const double ey = std::max(0.0, std::abs(angDiff(wantY, curAim[0])) - turnRate * 0.05);
            const double ep = std::max(0.0, std::abs(wantP - curAim[1]) - pitchRate * 0.05);
            return ey < tolY && ep < tolP;
        };

        for (int i = 0; i < 2; ++i) {
            const auto& ow = ws[static_cast<std::size_t>(i)];
            if (!ow) continue;
            const auto& w = *ow;
            if (w.kind == "gun") {
                if (gunOK && w.ammo > 0.0 && w.reload <= 0.0 && slewAligned(gAim[0], gAim[1])) {
                    fire[static_cast<std::size_t>(i)] = true;
                }
            } else if (w.kind == "lightning") {
                const double cone = radians(w.coneDeg);
                if (dist < w.range - 1.5 &&
                    std::abs(angDiff(hy, curAim[0])) < cone * 0.5 - 0.05 &&
                    energy >= w.energyCost) {
                    fire[static_cast<std::size_t>(i)] = true;
                }
            } else if (w.kind == "laser") {
                if (!isMech || !target || dist > w.range * 0.97) continue;
                if (now_ < shortUntil_ && w.charge <= 0.0) continue;
                const double full = w.fullCharge;
                if (p_.lethal && w.charge >= w.minCharge && w.charge < full - 0.051) {
                    double dmg = w.damage * (w.minFrac + (1.0 - w.minFrac) * (w.charge / full)) *
                        (1.0 + cfgDmgPerCube_ * static_cast<double>(std::min(myCubes, cfgCubeStatCap_)));
                    if (holdsKind(target->weapons, "mortar")) dmg *= (1.0 + w.cookoff);
                    if (dmg * p_.lethalK >= target->hp &&
                        std::abs(angDiff(hy, curAim[0])) < tolY * 0.9 &&
                        std::abs(hpPitch - curAim[1]) < tolP) {
                        aim = {hy, hpPitch};
                        fire[static_cast<std::size_t>(i)] = false;
                        continue;
                    }
                }
                if (w.charge < full - 0.051) {
                    fire[static_cast<std::size_t>(i)] = true;
                    if (w.charge > full * 0.45) {
                        aim = {hy, hpPitch};
                    }
                } else {
                    aim = {hy, hpPitch};
                    const double ey = std::abs(angDiff(hy, curAim[0]));
                    const double ep = std::abs(hpPitch - curAim[1]);
                    fire[static_cast<std::size_t>(i)] = !(ey < tolY * 0.9 && ep < tolP);
                }
            } else if (w.kind == "mortar") {
                if (!isMech || w.cooldown > 0.25 || dist < 16.0 || dist > 200.0) continue;
                const Vec3 tube{muzzle[0], muzzle[1], zf + ownHeight + 0.5};
                const Vec3 foot{tp[0], tp[1], tp[2] - th * 0.5};
                if (const auto ma = mortarAim(tube, foot, tv, w.speed, w.gravity, minPitch, maxPitch)) {
                    if (!gun || gun->reload > 0.0 || gun->ammo <= 0.0 || !gunOK) {
                        aim = *ma;
                    }
                    const double ey = std::max(0.0, std::abs(angDiff((*ma)[0], curAim[0])) - turnRate * 0.05);
                    const double ep = std::max(0.0, std::abs((*ma)[1] - curAim[1]) - pitchRate * 0.05);
                    if (ey < 0.04 && ep < 0.03) {
                        fire[static_cast<std::size_t>(i)] = true;
                    }
                }
            }
        }
    }

    // Free opportunistic lightning pass on any close mech right inside our 30-deg cone
    if (const int lSlot = findWeaponSlot(ws, "lightning"); lSlot >= 0 && !fire[static_cast<std::size_t>(lSlot)]) {
        const auto& lw = *ws[static_cast<std::size_t>(lSlot)];
        if (lw.cooldown <= 0.1 && energy >= lw.energyCost + 2.0) {
            const double halfCone = radians(lw.coneDeg * 0.5) - 0.05;
            for (const MechTrack* t : seen) {
                if (t->seen != now_ || !t->los || t->dist >= lw.range - 1.5) continue;
                const double yaw = std::atan2(t->pos[1] - y, t->pos[0] - x);
                if (std::abs(angDiff(yaw, curAim[0])) <= halfCone) {
                    fire[static_cast<std::size_t>(lSlot)] = true;
                    break;
                }
            }
        }
    }

    // Behind-wall laser pre-charge (peek mode) + brief 0.35s anti-misfire grace when target flickers behind cover
    if (!target && now_ >= shortUntil_) {
        for (int i = 0; i < 2; ++i) {
            if (ws[static_cast<std::size_t>(i)] && ws[static_cast<std::size_t>(i)]->kind == "laser") {
                const auto& lw = *ws[static_cast<std::size_t>(i)];
                if (foe && lw.cooldown <= 0.05 && foe->dist < lw.range &&
                    lw.charge < lw.fullCharge + lw.holdS - 0.35) {
                    fire[static_cast<std::size_t>(i)] = true;
                } else if (hidden && lw.charge >= lw.minCharge &&
                           lw.charge < lw.fullCharge + lw.holdS - 0.35 &&
                           now_ - hidden->seen <= 0.35 && hidden->dist < lw.range) {
                    fire[static_cast<std::size_t>(i)] = true;
                }
            }
        }
    }

    // Active Vision Scanning & Front-Armor Facing when not shooting
    if (!haveAim) {
        const MechTrack* watch = nullptr;
        for (const MechTrack* t : seen) {
            if (t->engage || now_ - t->hitMe < 6.0) {
                if (!watch || t->dist < watch->dist) watch = t;
            }
        }
        if (watch) {
            aim[0] = std::atan2(watch->pos[1] - y, watch->pos[0] - x);
        } else if (npcHurt_ > 0.0 && now_ - npcHurt_ < 2.5) {
            aim[0] = npcBearing_;
        } else if (now_ - lastHurt_ < 2.5) {
            aim[0] = hurtBearing_;
        } else if (mx != 0.0 || my != 0.0) {
            aim[0] = std::atan2(my, mx) + 1.15 * std::sin(now_ * 1.3);
        } else {
            aim[0] = curAim[0] + 0.35;
        }
        aim[1] = 0.0;
    }

    aim[0] = finiteOr(std::remainder(aim[0], kTau), lastAim_[0]);
    aim[1] = std::clamp(finiteOr(aim[1], 0.0), minPitch, maxPitch);
    lastAim_ = aim;

    // 7. Clean Early Reload (NEVER reloads mid-burst during inter-shot cooldown!)
    bool reload = false;
    if (gun && gun->reload <= 0.0 && gun->ammo < gun->mag) {
        if (!target && !farmNpc && !farmCratePos && (gun->ammo <= gun->mag * 0.6 || now_ - lastThreatAt_ > 3.0)) {
            reload = true;
        }
    }

    // Normalize move vector
    mx = finiteOr(mx, 0.0);
    my = finiteOr(my, 0.0);
    if (const double mn = std::hypot(mx, my); mn > 1.0) {
        mx /= mn;
        my /= mn;
    }

    Json action = {
        {"move", {mx, my}},
        {"aim", {aim[0], aim[1]}},
        {"boost", boost && (mx != 0.0 || my != 0.0)},
        {"jump", jump},
        {"fire", {fire[0], fire[1]}},
        {"reload", reload},
    };

    // 8. Explicit Weapon Pickup / Upgrade Swap (protocol.md §7)
    if (const auto pit = obs.find("pickups"); pit != obs.end() && pit->is_array()) {
        for (const auto& p : *pit) {
            if (!p.is_object() || !p.contains("id") || !p["id"].is_number()) continue;
            const Vec3 ppos = safeVec3(p, "pos");
            if (std::hypot(ppos[0] - x, ppos[1] - y) > cfgLootPickupRadius_ - 0.3 ||
                std::abs(ppos[2] - zf) > 3.6) {
                continue;
            }
            const std::string kind = p.value("kind", std::string{});
            if (!wantPickup(ws, kind)) continue;
            if (ws[1] && ws[0]) {
                if (ws[1]->kind == "laser" && ws[1]->charge > 0.0) break;
                action["drop"] = 1;
            }
            action["take"] = static_cast<int>(p["id"].get<double>());
            break;
        }
    }

    // 9. Upgrade Offer Selection
    if (const auto oit = obs.find("offer"); oit != obs.end() && !oit->is_null()) {
        action["pick"] = pickUpgrade(*oit);
    }

    if (cfgDebugMaxChars_ >= 16) {
        action["debug"] = {{"label", mode}};
    }
    return action;
}

}  // namespace mr
