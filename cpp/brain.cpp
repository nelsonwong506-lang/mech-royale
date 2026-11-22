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
constexpr double kTick = 0.05;
constexpr double kDecision = 2.0 * kTick;
constexpr int kProjectionDecisions = 32;
constexpr int kProjectionTicks = 2 * kProjectionDecisions;
constexpr int kProjectileTicks = 14;

double angDiff(double a, double b) {
    return std::remainder(a - b, kTau);
}

double numOr(const Json& obj, const char* key, double fallback) {
    if (!obj.is_object()) return fallback;
    const auto it = obj.find(key);
    if (it == obj.end() || !it->is_number()) return fallback;
    return finiteOr(it->get<double>(), fallback);
}

int intOr(const Json& obj, const char* key, int fallback) {
    if (!obj.is_object()) return fallback;
    const auto it = obj.find(key);
    if (it == obj.end() || !it->is_number()) return fallback;
    return static_cast<int>(finiteOr(it->get<double>(), static_cast<double>(fallback)));
}

bool boolOr(const Json& obj, const char* key, bool fallback) {
    if (!obj.is_object()) return fallback;
    const auto it = obj.find(key);
    if (it == obj.end() || !it->is_boolean()) return fallback;
    return it->get<bool>();
}

std::string strOr(const Json& obj, const char* key, const char* fallback = "") {
    if (!obj.is_object()) return fallback;
    const auto it = obj.find(key);
    if (it == obj.end() || !it->is_string()) return fallback;
    return it->get<std::string>();
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
        wi.kind = strOr(w, "kind");
        wi.cooldown = numOr(w, "cooldown", 0.0);
        wi.ammo = numOr(w, "ammo", 0.0);
        wi.reload = numOr(w, "reload", 0.0);
        wi.charge = numOr(w, "charge", 0.0);
        const auto& st = w.contains("stats") && w["stats"].is_object() ? w["stats"] : Json::object();
        if (wi.kind == "gun") {
            wi.damage = numOr(st, "damage", 45.0);
            wi.fireInterval = std::max(0.08, numOr(st, "cooldown", 0.24));
            wi.speed = numOr(st, "speed", 252.0);
            wi.range = numOr(st, "range", 230.0);
            wi.mag = std::max(1.0, numOr(st, "mag", 12.0));
            wi.reloadS = std::max(0.5, numOr(st, "reload", 2.4));
        } else if (wi.kind == "laser") {
            wi.damage = numOr(st, "damage", 315.0);
            wi.fireInterval = std::max(0.3, numOr(st, "cooldown", 1.65));
            wi.range = numOr(st, "range", 280.0);
            wi.fullCharge = std::max(0.15, numOr(st, "charge", 0.75));
            wi.minCharge = numOr(st, "min_charge", 0.15);
            wi.minFrac = numOr(st, "min_frac", 0.35);
            wi.holdS = numOr(st, "hold_s", 2.0);
            wi.cookoff = numOr(st, "cookoff", 0.6);
        } else if (wi.kind == "lightning") {
            wi.damage = numOr(st, "damage", 60.0);
            wi.fireInterval = std::max(0.08, numOr(st, "cooldown", 0.25));
            wi.range = numOr(st, "range", 45.0);
            wi.coneDeg = numOr(st, "cone_deg", 30.0);
            wi.energyCost = numOr(st, "energy", 3.0);
        } else if (wi.kind == "mortar") {
            wi.damage = numOr(st, "damage", 200.0);
            wi.fireInterval = std::max(0.5, numOr(st, "cooldown", 2.2));
            wi.speed = numOr(st, "speed", 72.0);
            wi.gravity = numOr(st, "gravity", 22.0);
            wi.splashRadius = numOr(st, "splash_radius", 7.0);
            wi.range = numOr(st, "range", wi.speed * wi.speed / std::max(1.0, wi.gravity));
        }
        out[i] = std::move(wi);
    }
    return out;
}

double gunHitProb(double dist) {
    const double u = dist / 155.0;
    return std::clamp(1.0 / (1.0 + u * u), 0.18, 0.96);
}

Vec2 limitLength(Vec2 v, double limit) {
    const double length = std::hypot(v[0], v[1]);
    if (length > limit) {
        v[0] *= limit / length;
        v[1] *= limit / length;
    }
    return v;
}

Vec2 toward(const Vec3& from, Vec2 to) {
    const Vec2 delta{to[0] - from[0], to[1] - from[1]};
    const double length = std::hypot(delta[0], delta[1]);
    return length > 1e-6 ? Vec2{delta[0] / length, delta[1] / length} : Vec2{};
}

double distanceTo(const Vec3& from, Vec2 to) {
    return std::hypot(to[0] - from[0], to[1] - from[1]);
}

}  // namespace

Brain::Brain(const Json& hello)
    : config_(hello.is_object() && hello.contains("config") && hello["config"].is_object()
                  ? hello["config"]
                  : Json::object()),
      arena_(hello.at("map").at("dir").get<std::string>(),
             hello.at("map").contains("files") && hello.at("map")["files"].is_array()
                 ? hello.at("map")["files"].get<std::vector<std::string>>()
                 : std::vector<std::string>{}),
      nav_(arena_, 2.6, 2.0),
      rng_(static_cast<std::uint32_t>(
          hello.contains("seed") && hello["seed"].is_number() && hello["seed"].get<std::uint64_t>() != 0
              ? hello["seed"].get<std::uint64_t>()
              : static_cast<std::uint64_t>(intOr(hello, "slot", 0) + 1))) {
    const auto& mech = config_.contains("mech") && config_["mech"].is_object() ? config_["mech"] : Json::object();
    cfgMechRadius_ = numOr(mech, "radius", 2.5);
    cfgMechHeight_ = numOr(mech, "height", 9.0);
    cfgMechEye_ = numOr(mech, "eye_height", 8.0);
    cfgMechMuzzle_ = numOr(mech, "muzzle_height", 7.0);
    cfgFrontArcDeg_ = numOr(mech, "front_arc_deg", 180.0);
    cfgRearMult_ = numOr(mech, "rear_mult", 1.5);
    cfgRegenDelay_ = numOr(mech, "regen_delay", 5.0);
    cfgRegenPerS_ = numOr(mech, "regen_per_s", 45.0);
    cfgEnergyRegen_ = std::max(1.0, numOr(mech, "energy_regen", 18.0));
    cfgMaxRiseSpeed_ = std::max(3.0, numOr(mech, "max_rise_speed", 15.0));

    const auto& npc = config_.contains("npc") && config_["npc"].is_object() ? config_["npc"] : Json::object();
    cfgNpcRadius_ = numOr(npc, "radius", 1.8);
    cfgNpcHeight_ = numOr(npc, "height", 3.5);

    const auto& cr = config_.contains("crates") && config_["crates"].is_object() ? config_["crates"] : Json::object();
    cfgCrateRadius_ = numOr(cr, "radius", 1.4);
    cfgCrateHeight_ = numOr(cr, "height", 2.4);

    const auto& cu = config_.contains("cubes") && config_["cubes"].is_object() ? config_["cubes"] : Json::object();
    cfgSizePerCube_ = numOr(cu, "size_per_cube", 0.05);
    cfgDmgPerCube_ = numOr(cu, "damage_per_cube", 0.04);
    cfgCubeStatCap_ = std::max(1, intOr(cu, "stat_cap", 10));
    if (const auto tit = cu.find("upgrade_thresholds"); tit != cu.end() && tit->is_array() && !tit->empty()) {
        cfgUpgradeThresholds_.clear();
        for (const auto& v : *tit) {
            if (v.is_number()) cfgUpgradeThresholds_.push_back(static_cast<int>(v.get<double>()));
        }
    }

    const auto& lt = config_.contains("loot") && config_["loot"].is_object() ? config_["loot"] : Json::object();
    cfgLootPickupRadius_ = numOr(lt, "pickup_radius", 4.0);

    const auto& weps = config_.contains("weapons") && config_["weapons"].is_object() ? config_["weapons"] : Json::object();
    const auto& mort = weps.contains("mortar") && weps["mortar"].is_object() ? weps["mortar"] : Json::object();
    cfgMortarGravity_ = numOr(mort, "gravity", 22.0);
    cfgMortarSplash_ = numOr(mort, "splash_radius", 7.0);

    if (const auto dit = config_.find("debug"); dit != config_.end() && dit->is_object()) {
        cfgDebugMaxChars_ = intOr(*dit, "max_text_chars", 0);
    }
    pings_.reserve(64);
    warEvents_.reserve(64);
    trapZones_.reserve(16);

    memNX_ = std::max(1, static_cast<int>(std::ceil((arena_.bounds[2] - arena_.bounds[0]) / memCellSize_)));
    memNY_ = std::max(1, static_cast<int>(std::ceil((arena_.bounds[3] - arena_.bounds[1]) / memCellSize_)));
    visitedAt_.assign(static_cast<std::size_t>(memNX_ * memNY_), -1e6f);
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

double Brain::soundHeat(double px, double py) const {
    double heat = 0.0;
    for (const auto& p : pings_) {
        const double age = now_ - p.at;
        if (age < 0.0 || age > 10.0) continue;
        const double d = std::hypot(px - p.pos[0], py - p.pos[1]);
        if (d >= p.radius) continue;
        heat += p.weight * std::exp(-age / 5.5) * (1.0 - d / p.radius);
    }
    return heat;
}

double Brain::warzoneHeat(double px, double py) const {
    double heat = 0.0;
    for (const auto& w : warEvents_) {
        const double age = now_ - w.at;
        if (age < 0.0 || age > 60.0) continue;
        const double d = std::hypot(px - w.pos[0], py - w.pos[1]);
        if (d >= 115.0) continue;
        heat += w.weight * std::exp(-age / 35.0) * (1.0 - d / 115.0);
    }
    return heat;
}

double Brain::trapPenalty(double px, double py) const {
    double pen = 0.0;
    for (const auto& tr : trapZones_) {
        const double age = now_ - tr.at;
        if (age < 0.0 || age > 90.0) continue;
        const double d = std::hypot(px - tr.pos[0], py - tr.pos[1]);
        if (d >= 35.0) continue;
        pen += tr.weight * std::exp(-age / 60.0) * (1.0 - d / 35.0) * 28.0;
    }
    return pen;
}

void Brain::markVisited(double px, double py) {
    const int ix = static_cast<int>((px - arena_.bounds[0]) / memCellSize_);
    const int iy = static_cast<int>((py - arena_.bounds[1]) / memCellSize_);
    if (ix < 0 || iy < 0 || ix >= memNX_ || iy >= memNY_) return;
    visitedAt_[static_cast<std::size_t>(iy * memNX_ + ix)] = static_cast<float>(now_);
}

double Brain::unvisitedScore(double px, double py) const {
    const int ix = static_cast<int>((px - arena_.bounds[0]) / memCellSize_);
    const int iy = static_cast<int>((py - arena_.bounds[1]) / memCellSize_);
    if (ix < 0 || iy < 0 || ix >= memNX_ || iy >= memNY_) return 0.0;
    const double last = static_cast<double>(visitedAt_[static_cast<std::size_t>(iy * memNX_ + ix)]);
    return std::clamp((now_ - last) / 60.0, 0.0, 1.0);
}

void Brain::recordTrap(double px, double py, double weight) {
    if (now_ - lastTrapAt_ < 2.5) return;
    lastTrapAt_ = now_;
    if (trapZones_.size() >= 16) {
        trapZones_.erase(trapZones_.begin());
    }
    trapZones_.push_back({{px, py}, now_, weight});
}

double Brain::regenBreakCost(double hpFrac, bool inZone) const {
    if (!inZone || hpFrac >= 0.94) return 0.0;
    const double since = now_ - lastCombatAt_;
    if (since < 1.8) return 0.0;
    const double progress = std::clamp(since / std::max(1.0, cfgRegenDelay_), 0.0, 1.25);
    const double missingFrac = std::clamp(1.0 - hpFrac, 0.0, 1.0);
    // Expected HP recovery lost by resetting the 5s timer right now (in cube-value units, 1 cube = 100)
    return missingFrac * 125.0 * progress;
}

double Brain::cubeValue(int cubes, double hpFrac) const {
    // 1. Base stat value (+4% maxHP + +4% damage up to stat_cap)
    double v = (cubes < cfgCubeStatCap_)
        ? (100.0 - 3.2 * static_cast<double>(cubes))
        : 10.0;

    // 2. Continuous upgrade-threshold sprint bonus (works for 1, 3, 6, 10, 15, 21...)
    int nextThr = -1;
    for (int thr : cfgUpgradeThresholds_) {
        if (thr > cubes) {
            nextThr = thr;
            break;
        }
    }
    if (nextThr > 0) {
        const int gap = nextThr - cubes;
        if (gap == 1) v += 85.0;
        else if (gap == 2) v += 42.0;
        else if (gap == 3) v += 18.0;
        else if (gap <= 5) v += 8.0;
    }

    // 3. Immediate heal bonus (+4% maxHP when below stat_cap)
    if (cubes < cfgCubeStatCap_) {
        v += 35.0 * std::clamp(1.0 - hpFrac, 0.0, 1.0);
    }

    // 4. Hitbox-enlargement penalty above stat_cap (no hard cutoff — lets safety/enemy-risk decide!)
    if (cubes >= cfgCubeStatCap_) {
        v -= 3.5 * static_cast<double>(cubes - cfgCubeStatCap_ + 1);
    }
    return std::max(8.0, v);
}

double Brain::weaponPower(std::string_view kind, double nextRadius) const {
    const double smallFrac = std::clamp((140.0 - nextRadius) / 90.0, 0.0, 1.0);
    if (kind == "laser") {
        // Dominant across open/mid arena; slightly tighter in tiny final circles
        return 190.0 - 25.0 * smallFrac;
    }
    if (kind == "lightning") {
        // Rises sharply in small circles (45m auto-hit + 2s laser short + chain 1vN)
        return 125.0 + 50.0 * smallFrac;
    }
    if (kind == "mortar") {
        // Offensive value (over-wall indirect fire + beats lightning) minus dynamic laser cook-off risk (+60% dmg!)
        const double offense = 78.0 - 22.0 * smallFrac;
        const double cookoffRisk = 128.0 * std::clamp(laserPresence_, 0.15, 1.0);
        return offense - cookoffRisk;
    }
    if (kind == "gun") {
        return 110.0;
    }
    return 0.0;
}

double Brain::weaponGain(const std::array<std::optional<WeaponInfo>, 2>& ws,
                         std::string_view kind, double nextRadius) const {
    if (kind == "gun") {
        return !ws[0].has_value() ? 140.0 : 0.0;
    }
    if (findWeaponSlot(ws, kind) >= 0) {
        return 0.0;
    }
    const double pNew = weaponPower(kind, nextRadius);
    if (!ws[1].has_value()) {
        // Empty secondary slot: only take if net positive after cook-off risk!
        return std::max(0.0, pNew);
    }
    const double pCur = weaponPower(ws[1]->kind, nextRadius);
    // Require meaningful net improvement to justify swap friction & 1s equip delay
    return std::max(0.0, pNew - pCur - 18.0);
}

bool Brain::wantPickup(const std::array<std::optional<WeaponInfo>, 2>& ws,
                       const std::string& kind, double nextRadius) const {
    return weaponGain(ws, kind, nextRadius) > 1.0;
}

int Brain::pickUpgrade(const Json& offer, const std::array<std::optional<WeaponInfo>, 2>& ws,
                       double nextRadius) const {
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

    const bool hasLaser = getWeapon(ws, "laser") != nullptr;
    const bool hasLight = getWeapon(ws, "lightning") != nullptr;
    const bool smallZone = nextRadius <= 90.0;

    auto scoreUpgrade = [&](std::string_view id) -> double {
        if (id == "plating") return 100.0;
        if (id == "armor") return 98.0;
        if (id == "nanites") return 90.0;
        if (id == "laser_damage") return hasLaser ? 95.0 : 72.0;
        if (id == "laser_charge") return hasLaser ? 93.0 : 70.0;
        if (id == "gun_damage") return 86.0;
        if (id == "gun_rate") return 84.0;
        if (id == "lightning_efficiency") return (hasLight && smallZone) ? 88.0 : (hasLight ? 80.0 : 60.0);
        if (id == "lightning_range") return (hasLight && smallZone) ? 87.0 : (hasLight ? 78.0 : 58.0);
        if (id == "servos") return 76.0;
        if (id == "gun_mag") return 71.0;
        if (id == "gun_velocity") return 69.0;
        if (id == "gyros") return 66.0;
        if (id == "lightning_chain") return hasLight ? 68.0 : 50.0;
        if (id == "reactor") return 64.0;
        if (id == "capacitor") return 62.0;
        if (id == "sensors") return 58.0;
        if (id == "jets") return 55.0;
        if (id == "mortar_reload") return 45.0;
        if (id == "mortar_splash") return 44.0;
        if (id == "mortar_velocity") return 42.0;
        if (id == "laser_pierce") return 38.0;
        return 30.0;
    };

    int best = 0;
    double bestScore = -1e9;
    for (std::size_t i = 0; i < choices->size(); ++i) {
        const auto& c = (*choices)[i];
        std::string_view id;
        if (c.is_object() && c.contains("id") && c["id"].is_string()) {
            id = c["id"].get_ref<const std::string&>();
        }
        const double sc = scoreUpgrade(id);
        if (sc > bestScore) {
            bestScore = sc;
            best = static_cast<int>(i);
        }
    }
    return best;
}

double Brain::myExpectedDPS(const std::array<std::optional<WeaponInfo>, 2>& ws,
                            int myCubes, double energy, const MechTrack& target,
                            double dist, bool rearShot, bool burst) const {
    const double cubeMult = 1.0 + cfgDmgPerCube_ * static_cast<double>(std::min(myCubes, cfgCubeStatCap_));
    const double rearMult = rearShot ? cfgRearMult_ : 1.0;
    const bool enemyMortar = holdsKind(target.weapons, "mortar");
    double dps = 0.0;

    for (const auto& ow : ws) {
        if (!ow) continue;
        const auto& w = *ow;
        if (w.kind == "gun") {
            if (dist > w.range * 0.96) continue;
            const double pHit = gunHitProb(dist);
            if (burst) {
                if (w.ammo > 0.0 && w.reload <= 0.20) {
                    dps += (w.damage / w.fireInterval) * pHit;
                }
            } else {
                const double cycle = w.mag * w.fireInterval + w.reloadS;
                dps += (w.damage * w.mag / cycle) * pHit;
            }
        } else if (w.kind == "laser") {
            if (dist > w.range * 0.97) continue;
            const double cook = enemyMortar ? (1.0 + w.cookoff) : 1.0;
            const double pHit = 0.88;
            if (burst) {
                if (w.charge >= w.fullCharge * 0.45) {
                    const double frac = w.minFrac + (1.0 - w.minFrac) * std::min(1.0, w.charge / w.fullCharge);
                    dps += (w.damage * frac * cook * pHit) / 0.65;
                } else if (w.cooldown <= 0.15) {
                    dps += (w.damage * cook * pHit) / (w.fullCharge + 0.9);
                }
            } else {
                dps += (w.damage * cook * pHit) / (w.fullCharge + w.fireInterval);
            }
        } else if (w.kind == "lightning") {
            if (dist >= w.range - 1.5 || energy < w.energyCost * 2.0) continue;
            dps += (w.damage / w.fireInterval) * (burst ? 1.0 : 0.88);
        } else if (w.kind == "mortar") {
            if (dist < 16.0 || dist > 200.0) continue;
            if (!burst || w.cooldown <= 0.30) {
                dps += (w.damage * 0.25) / std::max(1.0, w.fireInterval);
            }
        }
    }
    return dps * cubeMult * rearMult;
}

double Brain::enemyExpectedDPS(const MechTrack& enemy,
                               const std::array<std::optional<WeaponInfo>, 2>& myWs,
                               double myArmor, double myEnergy, double dist,
                               bool aimingAtMe, bool burst) const {
    if (enemy.kind == "npc") {
        return (dist < 90.0) ? 18.0 * (1.0 - myArmor) : 0.0;
    }
    const double cubeMult = 1.0 + cfgDmgPerCube_ * static_cast<double>(std::min(enemy.cubes, cfgCubeStatCap_));
    const bool iHaveMortar = getWeapon(myWs, "mortar") != nullptr;
    const bool iShortThem = (getWeapon(myWs, "lightning") != nullptr) && dist < 43.5 && myEnergy > 10.0;

    double dps = 0.0;
    const bool hasGun = (!enemy.weaponsKnown && enemy.weapons.empty()) || holdsKind(enemy.weapons, "gun");
    if (hasGun && dist < 225.0) {
        dps += 95.0 * gunHitProb(dist);
    }
    if (holdsKind(enemy.weapons, "laser") && dist < 275.0) {
        const double cook = iHaveMortar ? 1.6 : 1.0;
        const double shortFactor = iShortThem ? 0.20 : 1.0;
        double lDps = 120.0 * cook * shortFactor;
        if (burst && enemy.charging && aimingAtMe) {
            // Imminent hitscan beam! Huge burst threat so posUtility snaps behind cover immediately.
            lDps += 420.0 * cook * shortFactor;
        }
        dps += lDps;
    }
    if (holdsKind(enemy.weapons, "lightning") && dist < 46.0) {
        dps += 215.0;
    }
    if (holdsKind(enemy.weapons, "mortar") && dist >= 16.0 && dist <= 205.0) {
        dps += 32.0;
    }
    return dps * cubeMult * std::clamp(1.0 - myArmor, 0.25, 1.0);
}

double Brain::attentionWeight(const MechTrack& t, double x, double y) const {
    const double toMe = std::atan2(y - t.pos[1], x - t.pos[0]);
    const double off = std::abs(angDiff(t.aim[0], toMe));
    if (now_ - t.hitMe < 4.5 || t.dist < 42.0 || (t.charging && off < 0.35)) {
        return 1.0;
    }
    if (off < 0.28) return 0.95;
    if (off < 0.85) return 0.55;
    return 0.25;
}

double Brain::fireCommitment(const MechTrack& t, const Vec3& pos, double towardSpeed, double walkSpeed) const {
    const double bearing = std::atan2(pos[1] - t.pos[1], pos[0] - t.pos[0]);
    const double facing = 0.5 * (1.0 + std::cos(angDiff(t.aim[0], bearing)));
    const double approach = std::max(0.0, towardSpeed) / (walkSpeed + std::abs(towardSpeed));
    const double departure = std::max(0.0, -towardSpeed) / (walkSpeed + std::abs(towardSpeed));
    const double near = (1.0 - departure) / (1.0 + std::pow(t.dist / 32.0, 4.0));
    const double hit = std::clamp(1.0 - (now_ - t.hitMe) / 4.5, 0.0, 1.0);
    return 1.0 - (1.0 - facing * facing) * (1.0 - approach) * (1.0 - near) * (1.0 - hit);
}

double Brain::engageValue(const Json& obs, const Json& me,
                          const std::array<std::optional<WeaponInfo>, 2>& ws,
                          const MechTrack& t, double inDPS, bool late) const {
    const Vec3 pos = safeVec3(me, "pos");
    const double hp = std::max(1.0, numOr(me, "hp", 1400.0));
    const double maxHp = std::max(1.0, numOr(me, "max_hp", 1400.0));
    const double hpFrac = hp / maxHp;
    const double energy = numOr(me, "energy", 100.0);
    const bool inZone = boolOr(me, "in_zone", true);
    const int myCubes = intOr(me, "cubes", 0);
    const int alive = intOr(obs, "alive", 16);

    const bool rearShot = showsBack(t, pos[0], pos[1]);
    const double sustDPS = myExpectedDPS(ws, myCubes, energy, t, t.dist, /*rearShot=*/false, /*burst=*/false);
    const double openBurst = myExpectedDPS(ws, myCubes, energy, t, t.dist, rearShot, /*burst=*/true);
    if (std::max(sustDPS, openBurst) < 8.0) return -200.0;

    // 2-phase physical TTK: rear-armor opening only lasts ~1.0s before enemy turns around and fires back!
    const double openingTime = rearShot ? 1.0 : 0.0;
    const double openDmg = openBurst * openingTime;
    double ttkKill = 0.0;
    if (openDmg >= t.hp && openBurst > 15.0) {
        ttkKill = t.hp / openBurst;
    } else {
        const double remHp = std::max(0.0, t.hp - openDmg);
        const double frontBurst = myExpectedDPS(ws, myCubes, energy, t, t.dist, /*rearShot=*/false, /*burst=*/true);
        const double fastRemT = (frontBurst > 15.0) ? (remHp / frontBurst) : kInf;
        ttkKill = openingTime + ((fastRemT <= 2.2) ? fastRemT : (remHp / std::max(10.0, sustDPS)));
    }
    const auto& st = me.contains("stats") && me["stats"].is_object() ? me["stats"] : Json::object();
    const double myArmor = numOr(st, "armor", 0.0);
    // Once we shoot, the enemy turns and returns full duel DPS (not 0.25x passive attention!)
    const double enemyDuelDPS = enemyExpectedDPS(t, ws, myArmor, energy, t.dist, /*aimingAtMe=*/true, /*burst=*/false);
    const double effInDPS = (ttkKill <= openingTime + 0.15)
        ? std::max(12.0, inDPS)
        : std::max({12.0, inDPS, enemyDuelDPS});
    const double ttkDie = hp / effInDPS;
    const double ratio = ttkDie / std::max(0.25, ttkKill);
    const double pWin = 1.0 / (1.0 + std::exp(-2.1 * std::log(std::max(0.05, ratio))));

    // 1. Expected gain: dropped cubes (t.cubes + 2) + threat elimination value
    const double lootGain = std::min(5.0, static_cast<double>(t.cubes + 2)) * cubeValue(myCubes, hpFrac) * 0.48;
    const double toMe = std::atan2(pos[1] - t.pos[1], pos[0] - t.pos[0]);
    const bool aimedAtMe = std::abs(angDiff(t.aim[0], toMe)) < 0.30;
    double threatElim = 0.0;
    if (now_ - t.hitMe < 5.0) threatElim += 95.0;
    if (t.dist < 62.0) threatElim += (62.0 - t.dist) * 1.8;
    if (aimedAtMe && t.dist < 150.0) threatElim += 50.0;
    if (t.closing > 3.0 && t.dist < 140.0) threatElim += 30.0;
    if (late) threatElim += 140.0;
    if (alive <= 2) threatElim += 250.0;

    // 2. Expected HP loss cost & Battle Royale placement death penalty (score = 16 - place, kills give 0 bonus!)
    const double fightTime = std::min(ttkKill, 4.5);
    const double expDmg = std::min(hp, inDPS * fightTime * (t.los ? 1.0 : 0.7));
    const double survivalWeight = (120.0 + 11.0 * static_cast<double>(alive)) * (maxHp / std::max(hp, 260.0));
    const double hpLossCost = (expDmg / maxHp) * survivalWeight;

    const double deathPenalty = (pracOn_ || alive <= 2)
        ? 0.0
        : (120.0 + 22.0 * static_cast<double>(alive));
    const double unprovokedCost = (!pracOn_ && alive > 2 && now_ - t.hitMe >= 5.0 && t.dist >= 60.0)
        ? (45.0 + 0.35 * (t.dist - 60.0) + 5.0 * static_cast<double>(alive))
        : 0.0;

    // 3. Third-party sound/beam warzone risk & 5-second regen opportunity cost
    //    (If already under active enemy fire, enemy bullets will reset regen anyway -> discount regenCost!)
    const double thirdParty = late ? 0.0 : 20.0 * soundHeat(pos[0], pos[1]);
    const double regenDiscount = 1.0 / (1.0 + inDPS / 25.0);
    const double regenCost = (ttkKill < 1.5 || late) ? 0.0 : regenBreakCost(hpFrac, inZone) * regenDiscount;

    // 4. Target focus stickiness (avoid splitting DPS across targets)
    const double stick = (t.id == stickID_ && now_ - stickAt_ < 2.5) ? 45.0 : 0.0;

    // 5. Distance friction (gentle preference for closer/cleaner finishes)
    const double distCost = 0.22 * t.dist + (t.los ? 0.0 : 28.0);
    const double pracBonus = pracOn_ ? 55.0 : 0.0;

    return pWin * (lootGain + threatElim) - (1.0 - pWin) * deathPenalty - unprovokedCost
         - hpLossCost - thirdParty - regenCost + stick + pracBonus - distCost;
}

std::optional<double> Brain::bestArmorYaw(const Json& me, const ProjectileRisk& incoming,
                                          const std::array<std::optional<WeaponInfo>, 2>& ws,
                                          const std::vector<MechTrack*>& seen) const {
    const Vec3 pos = safeVec3(me, "pos");
    const double x = pos[0], y = pos[1];
    const auto& st = me.contains("stats") && me["stats"].is_object() ? me["stats"] : Json::object();
    const double myArmor = numOr(st, "armor", 0.0);
    const double energy = numOr(me, "energy", 100.0);

    struct ThreatDir {
        double angle;
        double weight;
    };
    std::vector<ThreatDir> dirs;
    dirs.reserve(24);

    auto addDir = [&](double ang, double w) {
        if (w <= 0.0 || !std::isfinite(ang)) return;
        ang = std::remainder(ang, kTau);
        if (ang < -std::numbers::pi) ang += kTau;
        dirs.push_back({ang, w});
    };

    for (const MechTrack* t : seen) {
        if (now_ - t->seen > 2.0) continue;
        if (t->weaponsKnown && t->weapons.empty()) continue;
        const double wAim = attentionWeight(*t, x, y);
        const bool activeThreat = t->engage || (now_ - t->hitMe < 5.0) || (t->dist < 75.0) ||
                                  (t->dist < 175.0 && (wAim >= 0.85 || (t->los && wAim >= 0.55)));
        if (!activeThreat) continue;
        const double ang = std::atan2(t->pos[1] - y, t->pos[0] - x);
        const double dps = enemyExpectedDPS(*t, ws, myArmor, energy, t->dist, wAim > 0.7, /*burst=*/true);
        const double losMult = t->los ? 1.0 : 0.35;
        addDir(ang, std::max(15.0, dps * wAim) * losMult);
    }

    if (incoming.damage > 0.0) addDir(incoming.bearing, 220.0);

    if (now_ - lastHurt_ < 2.5) {
        addDir(hurtBearing_, 180.0);
    }
    if (npcHurt_ > 0.0 && now_ - npcHurt_ < 2.5) {
        addDir(npcBearing_, 65.0);
    }
    if (now_ - lastLaserSoundAt_ < 1.8) {
        addDir(laserSoundBearing_, 150.0);
    }
    for (const auto& p : pings_) {
        if (p.exact && now_ - p.at < 1.5) {
            const double d = std::hypot(p.pos[0] - x, p.pos[1] - y);
            if (d < 130.0) {
                addDir(std::atan2(p.pos[1] - y, p.pos[0] - x), 130.0 * (1.0 - d / 145.0));
            }
        }
    }

    if (dirs.empty()) return std::nullopt;
    if (dirs.size() == 1) return dirs[0].angle;

    // Exact circular sliding-window maximum over front armor arc (default 180 deg = pi rad)
    std::sort(dirs.begin(), dirs.end(), [](const ThreatDir& a, const ThreatDir& b) {
        return a.angle < b.angle;
    });
    const std::size_t n = dirs.size();
    std::vector<ThreatDir> dup;
    dup.reserve(2 * n);
    for (const auto& d : dirs) dup.push_back(d);
    for (const auto& d : dirs) dup.push_back({d.angle + kTau, d.weight});

    const double arc = radians(std::clamp(cfgFrontArcDeg_, 60.0, 350.0)) - 0.08;
    double bestW = -1.0;
    double bestCenter = dirs[0].angle;
    double curW = 0.0;
    std::size_t r = 0;
    for (std::size_t l = 0; l < n; ++l) {
        while (r < l + n && dup[r].angle - dup[l].angle <= arc) {
            curW += dup[r].weight;
            ++r;
        }
        if (curW > bestW) {
            bestW = curW;
            // Center of covered span gives maximum margin on both sides of the 180-deg shield
            bestCenter = 0.5 * (dup[l].angle + dup[r - 1].angle);
        }
        curW -= dup[l].weight;
    }
    return std::remainder(bestCenter, kTau);
}

void Brain::observe(const Json& obs, const Json& me, const std::array<std::optional<WeaponInfo>, 2>& ws) {
    (void)ws;
    const Vec3 pos = safeVec3(me, "pos");
    const Vec3 vel = safeVec3(me, "vel");
    const auto& st = me.contains("stats") && me["stats"].is_object() ? me["stats"] : Json::object();
    const double eyeH = numOr(st, "eye_height", cfgMechEye_);
    const double muzH = numOr(st, "muzzle_height", cfgMechMuzzle_);
    if (st.contains("regen_delay") && st["regen_delay"].is_number()) {
        cfgRegenDelay_ = std::max(0.5, st["regen_delay"].get<double>());
    }
    const Vec3 eye{pos[0], pos[1], pos[2] + eyeH};
    const Vec3 muzzle{pos[0], pos[1], pos[2] + muzH};
    const int selfId = intOr(me, "id", -1);

    // Layer 1 Map Memory: mark current cell + swept 120-deg FOV cells as explored
    const Vec2 curAim = safeVec2(me, "aim");
    markVisited(pos[0], pos[1]);
    for (double d : {25.0, 55.0}) {
        for (double off : {-0.7, 0.0, 0.7}) {
            markVisited(pos[0] + std::cos(curAim[0] + off) * d,
                        pos[1] + std::sin(curAim[0] + off) * d);
        }
    }

    // Decay lobby laser prevalence gently (grounded in persistent roster + high prior while >4 players remain!)
    const int curAlive = intOr(obs, "alive", 16);
    int knownLasers = 0;
    for (const auto& [rid, re] : roster_) {
        (void)rid;
        if (!re.dead && re.hasLaser && now_ - re.lastSeen < 120.0) ++knownLasers;
    }
    const double rosterLaserFloor = (curAlive > 1)
        ? std::clamp(static_cast<double>(knownLasers) / static_cast<double>(curAlive - 1), 0.0, 1.0)
        : 0.0;
    const double minLaserPrior = std::max(rosterLaserFloor, (curAlive <= 4) ? 0.15 : 0.62);
    laserPresence_ = std::max(minLaserPrior, laserPresence_ * 0.997);

    if (const auto it = obs.find("mechs"); it != obs.end() && it->is_array()) {
        for (const auto& m : *it) {
            if (!m.is_object() || !m.contains("id") || !m["id"].is_number()) continue;
            const int id = static_cast<int>(m["id"].get<double>());
            if (id == selfId) continue;
            const std::string kind = strOr(m, "kind", "mech");
            auto& tab = (kind == "npc") ? npcs_ : tracks_;
            auto& t = tab[id];
            t.id = id;
            t.kind = kind;
            t.pos = safeVec3(m, "pos");
            t.vel = safeVec3(m, "vel");
            t.aim = safeVec2(m, "aim");
            t.hp = numOr(m, "hp", kind == "npc" ? 350.0 : 1400.0);
            t.maxHp = std::max(1.0, numOr(m, "max_hp", kind == "npc" ? 350.0 : 1400.0));
            t.cubes = intOr(m, "cubes", 0);
            t.charging = boolOr(m, "charging", false);
            const auto slots = m.find("weapons");
            const bool observedLoadout = slots != m.end() && slots->is_array();
            t.weapons.clear();
            t.weaponsKnown = observedLoadout;
            if (observedLoadout) {
                for (const auto& wk : *slots) {
                    if (wk.is_string()) {
                        const auto& s = wk.get_ref<const std::string&>();
                        t.weapons.push_back(s);
                        if (s == "laser") {
                            laserPresence_ = std::min(1.0, laserPresence_ + 0.12);
                        }
                    }
                }
            }
            // Layer 3 Map Memory: persistent opponent roster (never erased!)
            if (kind == "mech") {
                auto& re = roster_[id];
                re.dead = false;
                re.lastPos = t.pos;
                re.lastSeen = now_;
                re.cubes = std::max(re.cubes, t.cubes);
                if (observedLoadout) {
                    re.loadoutSeen = now_;
                    re.hasGun = holdsKind(t.weapons, "gun");
                    re.hasLaser = holdsKind(t.weapons, "laser");
                    re.hasLightning = holdsKind(t.weapons, "lightning");
                    re.hasMortar = holdsKind(t.weapons, "mortar");
                } else {
                    t.weaponsKnown = std::isfinite(re.loadoutSeen);
                    if (re.hasGun) t.weapons.push_back("gun");
                    if (re.hasLaser) t.weapons.push_back("laser");
                    if (re.hasLightning) t.weapons.push_back("lightning");
                    if (re.hasMortar) t.weapons.push_back("mortar");
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
                k.kind = strOr(item, "kind");
                k.pos = safeVec3(item, "pos");
                k.hp = numOr(item, "hp", 300.0);
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

    // Expire old threat pings (> 10s), long-term war events (> 60s), and trap zones (> 90s)
    pings_.erase(std::remove_if(pings_.begin(), pings_.end(),
                                [&](const ThreatPing& p) { return now_ - p.at > 10.0; }),
                 pings_.end());
    warEvents_.erase(std::remove_if(warEvents_.begin(), warEvents_.end(),
                                    [&](const WarEvent& w) { return now_ - w.at > 60.0; }),
                     warEvents_.end());
    trapZones_.erase(std::remove_if(trapZones_.begin(), trapZones_.end(),
                                    [&](const TrapZone& tr) { return now_ - tr.at > 90.0; }),
                     trapZones_.end());

    auto pushPing = [&](const ThreatPing& tp) {
        if (pings_.size() >= 48) {
            pings_.erase(pings_.begin());
        }
        pings_.push_back(tp);
    };
    auto pushWar = [&](const Vec2& wp, double ww, double at, bool isDeath) {
        if (warEvents_.size() >= 64) {
            warEvents_.erase(warEvents_.begin());
        }
        warEvents_.push_back({wp, ww, at, isDeath});
    };

    // 1. Exact beam origins (laser & lightning arcs from other mechs!)
    if (const auto bit = obs.find("beams"); bit != obs.end() && bit->is_array()) {
        for (const auto& bm : *bit) {
            if (!bm.is_object()) continue;
            const int by = intOr(bm, "by", -1);
            if (by == selfId) continue;
            const std::string kind = strOr(bm, "kind");
            const double age = numOr(bm, "age", 0.0);
            if (bm.contains("path") && bm["path"].is_array() && !bm["path"].empty()) {
                const auto& p0 = bm["path"][0];
                if (p0.is_array() && p0.size() >= 2 && p0[0].is_number() && p0[1].is_number()) {
                    const bool isL = (kind == "laser");
                    if (isL) laserPresence_ = std::min(1.0, laserPresence_ + 0.15);
                    const Vec2 bp{finiteOr(p0[0].get<double>()), finiteOr(p0[1].get<double>())};
                    pushPing({bp, isL ? 2.4 : 1.7, now_ - age, 80.0, isL, true});
                    pushWar(bp, isL ? 2.2 : 1.5, now_ - age, false);
                    if (by >= 0) {
                        auto& re = roster_[by];
                        if (now_ - age > re.loadoutSeen) {
                            if (isL) re.hasLaser = true;
                            else if (kind == "lightning") re.hasLightning = true;
                        }
                        re.lastPos = {bp[0], bp[1], pos[2]};
                        re.lastSeen = now_ - age;
                    }
                }
            }
        }
    }

    // 2. Sound intelligence (gun/laser/laser_charge/mortar/lightning/death/jet)
    if (const auto sit = obs.find("sounds"); sit != obs.end() && sit->is_array()) {
        for (const auto& s : *sit) {
            if (!s.is_object()) continue;
            const std::string kind = strOr(s, "kind");
            const std::string distStr = strOr(s, "dist", "mid");
            const double bearing = numOr(s, "bearing", 0.0);
            const double age = numOr(s, "age", 0.0);
            double estD = 105.0;
            if (distStr == "near") estD = 32.0;
            else if (distStr == "far") estD = 210.0;

            double w = 0.0;
            double rad = 75.0;
            bool isL = false;
            double warW = 0.0;
            bool isDeath = false;
            if (kind == "laser_charge") {
                w = (distStr == "near") ? 2.6 : 1.8;
                rad = 70.0;
                isL = true;
                warW = 1.6;
                laserPresence_ = std::min(1.0, laserPresence_ + 0.12);
                if (distStr != "far") {
                    lastLaserSoundAt_ = now_ - age;
                    laserSoundBearing_ = bearing;
                }
            } else if (kind == "laser") {
                w = 1.7;
                rad = 85.0;
                isL = true;
                warW = 1.6;
                laserPresence_ = std::min(1.0, laserPresence_ + 0.08);
            } else if (kind == "death") {
                w = 1.5;
                rad = 90.0;
                warW = 2.8;
                isDeath = true;
            } else if (kind == "gun" || kind == "lightning" || kind == "mortar" || kind == "explosion") {
                w = (distStr == "near") ? 1.4 : 0.9;
                rad = 75.0;
                warW = (kind == "explosion" || kind == "mortar") ? 1.8 : 0.9;
            } else if ((kind == "boost" || kind == "jet" || kind == "step") && distStr == "near") {
                w = 0.7;
                rad = 45.0;
            }
            if (w > 0.0) {
                const Vec2 sp{pos[0] + std::cos(bearing) * estD, pos[1] + std::sin(bearing) * estD};
                pushPing({sp, w, now_ - age, rad, isL, false});
                if (warW > 0.0 && distStr != "far") {
                    pushWar(sp, warW, now_ - age, isDeath);
                }
            }
        }
    }

    // 3. Events (exact 5-second combat regen clock: BOTH damaged AND hit reset regen!)
    if (const auto evIt = obs.find("events"); evIt != obs.end() && evIt->is_array()) {
        for (const auto& e : *evIt) {
            if (!e.is_object()) continue;
            const std::string type = strOr(e, "type");
            if (type == "damaged") {
                lastCombatAt_ = now_;
                const std::string wep = strOr(e, "weapon");
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
            } else if (type == "effect" && strOr(e, "effect") == "short") {
                const auto& kinds = config_.contains("weapons") ? config_["weapons"] : Json::object();
                const auto& arc = kinds.contains("lightning") ? kinds["lightning"] : Json::object();
                laserShortUntil_ = now_ + numOr(arc, "short_s", 2.0);
            } else if (type == "hit") {
                // Dealing damage to ANY target resets the 5s regen clock (protocol.md §7)
                lastCombatAt_ = now_;
            } else if (type == "kill") {
                ++kills_;
                if (e.contains("target") && e["target"].is_number()) {
                    const int tid = static_cast<int>(e["target"].get<double>());
                    tracks_.erase(tid);
                    roster_[tid].dead = true;
                }
            }
        }
    }

    // Automatic Practice (1v15 Squad) vs FFA detection
    const int alive = intOr(obs, "alive", 16);
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
    pracOn_ = !notPractice_ && now_ > 45.0;

    // 4. Zone phase anchor tracking
    if (const auto zit = obs.find("zone"); zit != obs.end() && zit->is_object()) {
        const auto& z = *zit;
        const double ss = numOr(z, "shrink_start", 0.0);
        const Vec2 c = safeVec2(z, "center");
        const double r = numOr(z, "radius", 560.0);
        if (holdStart_ != ss || now_ <= ss) {
            holdStart_ = ss;
            hold_ = {c[0], c[1], r};
        }
    }
}

ZoneCircle Brain::zoneAt(const Json& z, double t) const {
    const double ss = numOr(z, "shrink_start", 0.0);
    const double se = numOr(z, "shrink_end", 0.0);
    if (t <= ss || se <= ss) {
        return hold_;
    }
    const Vec2 nc = safeVec2(z, "next_center", {hold_.x, hold_.y});
    const double nr = numOr(z, "next_radius", hold_.r);
    const double f = std::clamp((t - ss) / (se - ss), 0.0, 1.0);
    return {hold_.x + (nc[0] - hold_.x) * f,
            hold_.y + (nc[1] - hold_.y) * f,
            hold_.r + (nr - hold_.r) * f};
}

ZoneCircle Brain::curZone(const Json& z) const {
    const Vec2 c = safeVec2(z, "center");
    const double r = numOr(z, "radius", 560.0);
    const ZoneCircle live{c[0], c[1], r};
    const double ss = numOr(z, "shrink_start", 0.0);
    if (now_ <= ss) return live;
    if (std::abs(live.r - hold_.r) > 0.01 || std::abs(live.x - hold_.x) > 0.01) {
        return live;
    }
    return zoneAt(z, now_);
}

double Brain::zoneDeadline(const Json& z, double x, double y, double pad) const {
    const Vec2 nc = safeVec2(z, "next_center");
    const double nr = numOr(z, "next_radius", 0.0);
    if (std::hypot(x - nc[0], y - nc[1]) <= nr - pad) {
        return kInf;
    }
    const double ss = numOr(z, "shrink_start", 0.0);
    const double se = numOr(z, "shrink_end", 0.0);
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
    const double nr = numOr(z, "next_radius", 0.0);
    if (const double dn = std::hypot(x - nc[0], y - nc[1]) - nr; dn > -5.0) {
        const double ss = numOr(z, "shrink_start", 0.0);
        const double soon = std::clamp(1.0 - (ss - now_) / 25.0, 0.1, 1.0);
        pen += soon * (25.0 + 0.8 * (dn + 5.0));
    }
    return pen;
}

Vec2 Brain::zoneGoal(const Json& z, double x, double y) const {
    const Vec2 nc = safeVec2(z, "next_center");
    const double nr = numOr(z, "next_radius", 0.0);
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

std::optional<Brain::ZoneDetour> Brain::zoneDetour(const Json& z, const FireState& state,
                                                 double speed, double accel, bool inZone) {
    const Vec3& pos = state.pos;
    const Vec2 start{pos[0], pos[1]};
    const ZoneCircle estimated = curZone(z);
    const Vec2 liveCenter = safeVec2(z, "center", {estimated.x, estimated.y});
    const ZoneCircle live{liveCenter[0], liveCenter[1], numOr(z, "radius", estimated.r)};
    const Vec2 center = safeVec2(z, "next_center", {live.x, live.y});
    const double radius = numOr(z, "next_radius", live.r);
    const double shrinkStart = numOr(z, "shrink_start", now_);
    const double shrinkEnd = numOr(z, "shrink_end", now_);
    const double pad = std::min(radius * 0.5, 8.0);
    const double safeRadius = radius - pad;
    const double radialDistance = distanceTo(pos, center);
    if (safeRadius < nav_.cellStep() || radialDistance <= safeRadius ||
        nav_.onRoofObst(pos[0], pos[1], pos[2])) return std::nullopt;

    const double directDistance = radialDistance - safeRadius;
    // During the hold, defer the search if even a long detour has ample time.
    if (inZone && shrinkStart - now_ > directDistance / speed + 35.0) return std::nullopt;

    const Vec2 radial{center[0] + (pos[0] - center[0]) * safeRadius / radialDistance,
                      center[1] + (pos[1] - center[1]) * safeRadius / radialDistance};
    auto walkableFromHere = [&](const Vec2& point) {
        return nav_.lineWalkable(start[0], start[1], point[0], point[1]);
    };
    if (walkableFromHere(radial)) return std::nullopt;

    const bool samePhase = detourPhaseEnd_ == shrinkEnd && detourCircle_.x == center[0] &&
                           detourCircle_.y == center[1] && detourCircle_.r == radius;
    constexpr std::size_t segmentCells = 14;
    const double attachDistance = (segmentCells * std::sqrt(2.0) + 1.0) * nav_.cellStep();
    auto attach = [&]() {
        const auto end = std::min(detourIndex_ + 3, detourPath_.size());
        for (std::size_t i = end; i > detourIndex_; --i) {
            const Vec2& point = detourPath_[i - 1];
            const double distance = distanceTo(pos, point);
            if (distance > 0.5 && distance <= attachDistance && walkableFromHere(point)) {
                detourIndex_ = i - 1;
                return true;
            }
        }
        return false;
    };
    if (!samePhase || !attach()) {
        if (samePhase && now_ < detourRetryAt_) return std::nullopt;
        detourCircle_ = {center[0], center[1], radius};
        detourPhaseEnd_ = shrinkEnd;
        detourRetryAt_ = now_ + 1.0;
        detourIndex_ = 0;
        detourPath_.clear();
        const auto [path, reached] = nav_.pathToCircle(start, center, safeRadius, 9000);
        if (!reached || path.empty()) return std::nullopt;
        for (std::size_t i = 0;;) {
            const Vec2 point = nav_.cellPos(path[i]);
            detourPath_.push_back(point);
            if (i + 1 == path.size()) break;
            std::size_t next = std::min(i + segmentCells, path.size() - 1);
            for (; next > i + 1; --next) {
                const Vec2 end = nav_.cellPos(path[next]);
                if (nav_.lineWalkable(point[0], point[1], end[0], end[1])) break;
            }
            i = next;
        }
        if (!attach()) return std::nullopt;
    }

    const double motionStart = std::max(now_, shrinkStart);
    auto exitAt = [&](Vec2 point) {
        if (std::hypot(point[0] - center[0], point[1] - center[1]) <= safeRadius) return kInf;
        if (std::hypot(point[0] - live.x, point[1] - live.y) >= live.r - pad) return motionStart;
        double low = 0.0, high = 1.0;
        for (int i = 0; i < 14; ++i) {
            const double f = 0.5 * (low + high);
            const double dx = point[0] - std::lerp(live.x, center[0], f);
            const double dy = point[1] - std::lerp(live.y, center[1], f);
            if (std::hypot(dx, dy) <= std::lerp(live.r, radius, f) - pad) low = f;
            else high = f;
        }
        return std::lerp(motionStart, std::max(motionStart, shrinkEnd), low);
    };

    double latest = exitAt(start);
    double elapsed = 0.0;
    Vec2 from = start;
    Vec2 velocity{state.vel[0], state.vel[1]};
    for (std::size_t i = detourIndex_; i < detourPath_.size(); ++i) {
        const Vec2 to = detourPath_[i];
        const double distance = std::hypot(to[0] - from[0], to[1] - from[1]);
        if (distance <= 1e-6) continue;
        const Vec2 direction{(to[0] - from[0]) / distance, (to[1] - from[1]) / distance};
        const double initial = std::clamp(velocity[0] * direction[0] + velocity[1] * direction[1], -speed, speed);
        const double rampDistance = (speed * speed - initial * initial) / (2.0 * accel);
        auto travelTime = [&](double length) {
            if (length > rampDistance) return (speed - initial) / accel + (length - rampDistance) / speed;
            return (std::sqrt(initial * initial + 2.0 * accel * length) - initial) / accel;
        };
        // Check exposure along the route. A wall can force an outward leg before safe entry.
        const int samples = static_cast<int>(std::ceil(distance / (2.0 * nav_.cellStep())));
        for (int j = 1; j <= samples; ++j) {
            const double f = static_cast<double>(j) / samples;
            const Vec2 sample{std::lerp(from[0], to[0], f), std::lerp(from[1], to[1], f)};
            latest = std::min(latest, exitAt(sample) - elapsed - travelTime(f * distance));
        }
        elapsed += travelTime(distance);
        const double endSpeed = std::min(speed, std::sqrt(initial * initial + 2.0 * accel * distance));
        velocity = {direction[0] * endSpeed, direction[1] * endSpeed};
        from = to;
    }
    return ZoneDetour{detourPath_[detourIndex_], latest};
}

bool Brain::runEscape(const Json& me, double& mx, double& my, bool& boost, bool& jump) {
    const Vec3 pos = safeVec3(me, "pos");
    const double x = pos[0], y = pos[1], zf = pos[2];
    const bool grounded = boolOr(me, "grounded", true);
    const double energy = numOr(me, "energy", 100.0);
    const double maxEnergy = std::max(1.0, numOr(me, "max_energy", 100.0));

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
    const double rad = cfgMortarSplash_ + numOr(st, "radius", cfgMechRadius_) + 2.0;

    for (const auto& pr : *pit) {
        if (!pr.is_object() || strOr(pr, "kind") != "mortar") continue;
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

std::optional<Vec2> Brain::findRoofLaunch(double ix, double iy, double iz, double ownHeight) const {
    // Search rings of 4m..16m around the rooftop item for the closest MainComp launch cell with clear ceiling
    std::optional<Vec2> best;
    double bestD = kInf;
    for (double r : {5.0, 8.0, 11.5, 15.0}) {
        for (int k = 0; k < 12; ++k) {
            const double a = k * (kTau / 12.0);
            const double lx = ix + std::cos(a) * r;
            const double ly = iy + std::sin(a) * r;
            if (!nav_.inBounds(lx, ly) || !nav_.reachable(lx, ly)) continue;
            const double lz = arena_.walkRaster(lx, ly);
            if (iz - lz > 44.0) continue;
            if (!arena_.ceilingClear(lx, ly, lz + 1.0, iz + ownHeight)) continue;
            const double d = std::hypot(lx - ix, ly - iy);
            if (d < bestD) {
                bestD = d;
                best = {lx, ly};
            }
        }
        if (best) return best;
    }
    return std::nullopt;
}

Vec2 Brain::fightMove(const Json& obs, const Json& me, const std::array<std::optional<WeaponInfo>, 2>& ws,
                      const MechTrack& tg, const std::vector<MechTrack*>& seen) {
    const Vec3 pos = safeVec3(me, "pos");
    const double x = pos[0], y = pos[1];
    const auto& st = me.contains("stats") && me["stats"].is_object() ? me["stats"] : Json::object();
    const double mh = numOr(st, "muzzle_height", cfgMechMuzzle_);
    const double ownH = numOr(st, "height", cfgMechHeight_);
    const double myArmor = numOr(st, "armor", 0.0);
    const double hp = std::max(1.0, numOr(me, "hp", 1400.0));
    const double maxHp = std::max(1.0, numOr(me, "max_hp", 1400.0));
    const double energy = numOr(me, "energy", 100.0);
    const int myCubes = intOr(me, "cubes", 0);
    const int alive = intOr(obs, "alive", 16);
    const auto& z = obs.contains("zone") && obs["zone"].is_object() ? obs["zone"] : Json::object();

    // Separate primary-duel weight (never refuses fair 1v1!) vs heavy 3rd-party cover-splitting!
    const double hpFrac = hp / maxHp;
    const WeaponInfo* myLaserW = getWeapon(ws, "laser");
    const bool laserHot = myLaserW && (myLaserW->charge >= myLaserW->fullCharge * 0.45);
    const double wPrimary = 0.46 + 0.44 * std::clamp(1.0 - hpFrac, 0.0, 1.0);
    const double lambdaExtra = 1.55 + 0.05 * static_cast<double>(alive);

    // Physical optimal matchup range (accounts for 0.25s bullet-dodge flight time & weapon ranges!)
    const bool iHaveLight = (getWeapon(ws, "lightning") != nullptr) && energy > 12.0;
    const bool iHaveLaser = getWeapon(ws, "laser") != nullptr;
    double optRange = 85.0;
    if (tg.kind == "npc") {
        optRange = 58.0;
    } else if (iHaveLight) {
        optRange = 28.0;
    } else if (iHaveLaser) {
        optRange = pracOn_ ? 115.0 : 88.0;
    }
    if (tg.kind != "npc" && !iHaveLight) {
        if (holdsKind(tg.weapons, "lightning")) optRange = std::max(optRange, 72.0);
        if (holdsKind(tg.weapons, "mortar")) optRange = std::max(optRange, 80.0);
    }
    if (tg.kind != "npc" && tg.hp < 0.35 * tg.maxHp && hpFrac > 0.48 && (pracOn_ || alive <= 2)) {
        optRange = std::min(optRange, 28.0);
    }

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

    for (int k = -1; k < 32; ++k) {
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
        const double cz = arena_.walkRaster(cx, cy);
        const double d = std::hypot(tg.pos[0] - cx, tg.pos[1] - cy);
        const bool rearShot = showsBack(tg, cx, cy);
        const bool los = !arena_.blocksLine({cx, cy, cz + mh}, tc);

        // 1. Instantaneous offensive output at (cx, cy)
        const double myBurst = los ? myExpectedDPS(ws, myCubes, energy, tg, d, rearShot, /*burst=*/true) : 0.0;

        // 2. Separate primary-target DPS, extra 3rd-party enemy DPS (1vMany cover-splitting!), and charging-laser dodge
        const Vec3 body{cx, cy, cz + ownH * 0.6};
        double primaryIn = 0.0;
        double extraIn = 0.0;
        double laserDanger = 0.0;
        bool tgSeesMe = false;
        for (const MechTrack* e : seen) {
            const double de = std::hypot(e->pos[0] - cx, e->pos[1] - cy);
            if (de > 225.0) continue;
            const Vec3 eEye = eyeOf(*e);
            if (arena_.blocksLine(eEye, body)) continue;
            if (e->id == tg.id) tgSeesMe = true;
            const double wAim = attentionWeight(*e, cx, cy);
            double edps = enemyExpectedDPS(*e, ws, myArmor, energy, de, wAim > 0.7, /*burst=*/false) * wAim;
            // Tree canopy one-way stealth: >45m enemy cannot see us through dense foliage while shots pierce!
            const bool canopyHidden = (de > 48.0 && arena_.blocksVisionTree(eEye, body));
            if (canopyHidden) {
                edps *= 0.15;
            }
            if (e->id == tg.id) {
                primaryIn += edps;
            } else {
                extraIn += edps;
            }
            if (!canopyHidden && e->charging && wAim > 0.7) {
                laserDanger += 320.0;
            }
        }
        if (tg.kind == "npc" && !tgSeesMe && los && d < 85.0) {
            primaryIn += 14.0 * (1.0 - myArmor);
        }

        // Close-range dodge collapse: below 58m enemy bullets arrive in <0.23s (too fast to sidestep!)
        if (!iHaveLight && tg.kind != "npc" && d < 58.0) {
            primaryIn *= 1.0 + 0.80 * (58.0 - d) / 58.0;
        }

        // Core physics net combat rate: actively fights 1v1 when weapon ready (uncapped peek bonus when Laser is charged!),
        // splits angles vs 2nd/3rd foes, automatically hides during reload/cooldown (myBurst==0) or enemy charging laser!
        const double fireVal = std::min(myBurst, iHaveLight ? 240.0 : 145.0) + (los && laserHot ? 140.0 : 0.0);
        double s = fireVal - wPrimary * primaryIn - lambdaExtra * extraIn - laserDanger - r * 0.6;

        // 3. Continuous optimal-range & weapon-matchup potential (keeps Gun/Laser at ~85m kiting range,
        //    Lightning at 28m, strictly outside enemy Lightning/Mortar!)
        s -= 1.45 * std::abs(d - optRange);
        const double mySust = myExpectedDPS(ws, myCubes, energy, tg, d, rearShot, /*burst=*/false);
        const double tgSust = enemyExpectedDPS(tg, ws, myArmor, energy, d, true, /*burst=*/false);
        s += 0.12 * (mySust - 0.95 * tgSust);

        // 4. Peek-corner geometry bonus (hug corners on the active side where 1 step toggles LOS)
        const bool wantLos = (myBurst > 20.0 && laserDanger <= 0.0);
        if (k >= 0 && k < 16 && (los == wantLos)) {
            for (int q = 0; q < 16; q += 2) {
                const double qa = q * (std::numbers::pi / 8.0);
                const double qx = cx + std::cos(qa) * 7.0, qy = cy + std::sin(qa) * 7.0;
                if (nav_.free(qx, qy) &&
                    (!arena_.blocksLine({qx, qy, arena_.walkRaster(qx, qy) + mh}, tc)) != los) {
                    s += 14.0;
                    break;
                }
            }
        }

        // 5. Rear-armor flank angle bonus + zone + wall clearance + strafe + nearby cube pull
        if (rearShot && d < 140.0) s += 16.0;
        s -= zonePenalty(z, cx, cy) * 1.4;
        if (nearCube) {
            s += 2.2 * (nearCubeD - std::hypot(nearCube->pos[0] - cx, nearCube->pos[1] - cy));
        }
        const double cl = nav_.clearanceAt(cx, cy);
        if (cl < 4.0) {
            s -= (4.0 - cl) * 4.5;
        }
        if (myBurst > 20.0) {
            s += 16.0 * (dx * sx + dy * sy);
        }
        if (k < 0) {
            // Slight preference to stay tucked if already hidden while cooling down/reloading
            if (myBurst <= 5.0 && (primaryIn + extraIn) <= 1.0) s += 14.0;
            else s -= 10.0;
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
    const double ownH = numOr(st, "height", cfgMechHeight_);
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
        const double cz = arena_.walkRaster(cx, cy);
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
        x, y, hidden,
        [&](double cx, double cy) {
            return zonePenalty(z, cx, cy) + 15.0 * std::min(soundHeat(cx, cy), 2.0) + trapPenalty(cx, cy);
        },
        threatXY, 50, 350);
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
                                                   double hpFrac, bool inZone, bool incomingRound) {
    const Vec3 pos = safeVec3(me, "pos");
    const double x = pos[0], y = pos[1], zf = pos[2];
    const auto& z = obs.contains("zone") && obs["zone"].is_object() ? obs["zone"] : Json::object();
    const auto& st = me.contains("stats") && me["stats"].is_object() ? me["stats"] : Json::object();
    const double walkSpeed = std::max(1.0, numOr(st, "walk_speed", 11.7));
    const double ownH = numOr(st, "height", cfgMechHeight_);
    const double energy = numOr(me, "energy", 100.0);
    const double maxEnergy = std::max(1.0, numOr(me, "max_energy", 100.0));
    const double myHp = std::max(1.0, numOr(me, "hp", 1400.0));
    const int myCubes = intOr(me, "cubes", 0);
    const int selfId = intOr(me, "id", -1);
    const int alive = intOr(obs, "alive", 16);
    const double nextRadius = numOr(z, "next_radius", 380.0);
    const ZoneCircle cur = curZone(z);

    // Estimate our gun DPS for crate/NPC time calculation
    const WeaponInfo* gun = getWeapon(ws, "gun");
    const double cubeMult = 1.0 + cfgDmgPerCube_ * static_cast<double>(std::min(myCubes, cfgCubeStatCap_));
    const double farmDPS = gun ? (gun->damage / gun->fireInterval) * cubeMult : 120.0;
    const double regenCost = regenBreakCost(hpFrac, inZone);

    // Expected weapon gain from opening a crate (50% chance, equal weights laser/lightning/mortar)
    const double avgCrateWeaponGain = 0.5 * (
        weaponGain(ws, "laser", nextRadius) +
        weaponGain(ws, "lightning", nextRadius) +
        weaponGain(ws, "mortar", nextRadius)
    ) / 3.0;

    // Continuous minimum threshold (higher when low HP so we don't wander far for scraps)
    const double minThreshold = 2.0 + 4.2 * std::clamp(1.0 - hpFrac, 0.0, 1.0);

    std::optional<LootChoice> best;
    double bestU = minThreshold;

    auto evaluate = [&](const std::string& kind, int id, const Vec3& ipos,
                        double netGain, double actTime, double extraRisk) {
        if (netGain <= 1.0) return;
        if (auto bi = badLoot_.find(id); bi != badLoot_.end() && bi->second > now_) return;

        const double d = std::hypot(ipos[0] - x, ipos[1] - y);
        const bool isNoisy = (kind == "crate" || kind == "npc");
        const double maxLootD = (kind == "pickup" && netGain >= 140.0) ? 200.0
                              : (kind == "npc") ? (pracOn_ ? 140.0 : 88.0)
                              : (kind == "crate") ? (pracOn_ ? 150.0 : 95.0)
                              : (pracOn_ ? 160.0 : 120.0);
        if (d > maxLootD) return;
        if (std::hypot(ipos[0] - cur.x, ipos[1] - cur.y) > cur.r - 5.5) return;

        // Rooftop vs ground reachability & climb physics
        const double groundAtItem = arena_.ground(ipos[0], ipos[1]);
        const bool itemOnRoof = (ipos[2] - groundAtItem > 3.2);
        bool needClimb = false;
        Vec2 launch{};
        double climbEnergy = 0.0;
        double roofTime = 0.0;

        if (itemOnRoof) {
            const double dz = ipos[2] - zf;
            if (dz > 1.5) {
                needClimb = true;
                // Only expose ourselves on a rooftop for high-value weapon pickups (Laser / empty-slot Lightning),
                // low vertical rise (<= 13m), short approach (<= 55m), and ample energy!
                if (kind != "pickup" || netGain < 130.0) return;
                climbEnergy = std::max(0.0, (dz - 2.0) / 0.55);
                if (dz > 13.0 || climbEnergy + 28.0 > maxEnergy || d > 55.0) return;
                const auto lOpt = findRoofLaunch(ipos[0], ipos[1], ipos[2], ownH);
                if (!lOpt) return;
                launch = *lOpt;
                const double waitE = std::max(0.0, climbEnergy + 14.0 - energy) / cfgEnergyRegen_;
                roofTime = waitE + dz / (cfgMaxRiseSpeed_ * 0.75) + 1.2;
            }
        } else {
            if (!nav_.isMainReachable(ipos[0], ipos[1])) return;
        }

        const double moveTime = (d * 1.25) / walkSpeed;
        const double totalTime = moveTime + actTime + roofTime;

        // Hard zone deadline check
        const double dl = zoneDeadline(z, ipos[0], ipos[1], 6.0);
        if (now_ + totalTime + 4.5 > dl) return;

        // Continuous EnemySafety(pos, pathMidpoint) — wider awareness (175m) for noisy crates/NPCs that alert hunters!
        const double midX = 0.5 * (x + ipos[0]), midY = 0.5 * (y + ipos[1]);
        const double threatRad = isNoisy ? 175.0 : 125.0;
        double enemyRisk = 0.0;
        double nearEnemyThreat = 0.0;
        for (const auto& [tid, t] : tracks_) {
            (void)tid;
            if (now_ - t.seen > 15.0) continue;
            const double de = std::hypot(t.pos[0] - ipos[0], t.pos[1] - ipos[1]);
            const double dm = std::hypot(t.pos[0] - midX, t.pos[1] - midY);
            const double dMin = std::min(de, dm);
            if (dMin > threatRad) continue;
            const double theirEdge = (t.hp / myHp) * (1.0 + 0.04 * static_cast<double>(t.cubes - myCubes));
            // Unwinnable contest right on the item?
            if (de < d * 0.78 && de < 65.0 && theirEdge > 0.72) {
                return;
            }
            const double prox = std::clamp((threatRad - dMin) / threatRad, 0.0, 1.0);
            const double contrib = prox * prox * std::clamp(theirEdge, 0.45, 2.0) * (isNoisy ? 2.2 : 1.6);
            enemyRisk += contrib;
            if (dMin < 95.0) nearEnemyThreat += contrib;
        }
        // Layer 3 Map Memory: long-term roster awareness (15s..45s) for strong foes last seen near this item
        for (const auto& [rid, re] : roster_) {
            (void)rid;
            const double age = now_ - re.lastSeen;
            if (re.dead || age <= 15.0 || age > 45.0) continue;
            if (!re.hasLaser && !re.hasLightning && re.cubes < 4) continue;
            const double dr = std::hypot(re.lastPos[0] - ipos[0], re.lastPos[1] - ipos[1]);
            if (dr < 95.0) {
                enemyRisk += 0.45 * std::exp(-(age - 15.0) / 20.0) * (1.0 - dr / 95.0);
            }
        }

        const double heat = soundHeat(ipos[0], ipos[1]);
        const double wHeat = std::min(warzoneHeat(ipos[0], ipos[1]), 2.5);
        if (needClimb && (nearEnemyThreat > 0.15 || heat > 0.35 || wHeat > 1.2)) {
            return;
        }
        const double expo = nav_.exposure(ipos[0], ipos[1]);
        const double expoRisk = expo * (0.35 + 1.35 * std::clamp(1.0 - hpFrac, 0.0, 1.0));
        const double airRisk = needClimb ? (roofTime * (0.65 * nearEnemyThreat + 0.35 * heat + 0.10)) : 0.0;
        const double warRisk = (isNoisy ? 0.38 : 0.22) * wHeat;
        const double safety = std::exp(-(enemyRisk + 0.65 * heat + warRisk + expoRisk + airRisk + extraRisk));

        double u = (netGain / (totalTime + 1.8)) * safety;
        if (id == lootID_) u *= 1.25;  // Anti-oscillation hysteresis

        if (u > bestU) {
            bestU = u;
            best = LootChoice{kind, id, ipos, u, needClimb, launch, climbEnergy};
        }
    };

    // 1. Ground & rooftop weapon pickups
    for (const auto& [pid, p] : pickups_) {
        (void)pid;
        const double gain = weaponGain(ws, p.kind, nextRadius);
        if (gain > 1.0) {
            evaluate("pickup", p.id, p.pos, gain, 0.0, 0.0);
        }
    }

    // 2. Power cubes (supports 10 -> 15 -> 21 upgrades seamlessly via cubeValue!)
    const double cVal = cubeValue(myCubes, hpFrac);
    for (const auto& [cid, c] : cubes_) {
        (void)cid;
        evaluate("cube", c.id, c.pos, cVal, 0.0, 0.0);
    }

    // Unified regen-silence & danger gate: never shoot loud crates/NPCs while hurt OR within 6s of recent threat!
    const bool protectRegen = incomingRound || (regenCost > 28.0) ||
                              (!pracOn_ && (hpFrac < 0.70 || now_ - lastThreatAt_ < 6.0));
    // Player-density gunshot reveal cost: shooting crates/NPCs in shrinking circles alerts the entire lobby!
    const double zoneScale = std::max(0.65, cur.r / 150.0);
    const double playerDensity = static_cast<double>(alive) / (zoneScale * zoneScale);
    const double gearedMult = (!pracOn_ && ws[1].has_value() && myCubes >= 3) ? 2.1 : 1.0;
    const double densityNoiseCost = pracOn_ ? 0.0 : (16.0 * playerDensity * gearedMult);

    // 3. Loot crates (1 cube + 50% weapon chance - noise/density/regen opportunity cost)
    if (!protectRegen) {
        const double crateNoiseCost = 28.0 + densityNoiseCost + regenCost;
        for (const auto& [cid, c] : crates_) {
            (void)cid;
            const double chp = c.hp > 1.0 ? c.hp : 300.0;
            const double breakT = chp / std::max(50.0, farmDPS) + 0.8;
            const double net = cVal + avgCrateWeaponGain - crateNoiseCost;
            evaluate("crate", c.id, c.pos, net, breakT, 0.30);
        }

        // 4. NPC drones (2 cubes - counter-damage & loud 200m+320m firefight reveal cost)
        const double twoCubeVal = cubeValue(myCubes, hpFrac) + cubeValue(myCubes + 1, hpFrac);
        const double npcCounterCost = 62.0 + densityNoiseCost * 1.4 + 120.0 * std::clamp(1.0 - hpFrac, 0.0, 1.0) + regenCost;
        for (const auto& [nid, n] : npcs_) {
            (void)nid;
            if (now_ - n.seen > 15.0) continue;
            const double killT = std::max(1.0, n.hp) / std::max(50.0, farmDPS) + 1.5;
            const double net = twoCubeVal - npcCounterCost;
            evaluate("npc", n.id, n.pos, net, killT, 0.55);
        }
    }

    // 5. Supply drops (3 cubes + full heal + full energy vs global-beacon contest/crowd risk!)
    if (const auto dit = obs.find("drops"); dit != obs.end() && dit->is_array()) {
        for (const auto& dr : *dit) {
            if (!dr.is_object()) continue;
            const int did = intOr(dr, "id", 20000);
            const Vec3 dpos = safeVec3(dr, "pos");
            const double landsIn = numOr(dr, "lands_in", 25.0);
            const double capS = numOr(dr, "capture_s", 4.0);
            const double prog = numOr(dr, "progress", 0.0);
            const int holder = intOr(dr, "holder", -1);
            const double d = std::hypot(dpos[0] - x, dpos[1] - y);
            const double moveT = (d * 1.25) / walkSpeed;

            // Unwinnable capture race? Skip immediately.
            if (holder >= 0 && holder != selfId && prog > 0.8 && moveT > (capS - prog) - 0.4) {
                continue;
            }
            const double waitLand = std::max(0.0, landsIn - moveT);
            const double remCap = (holder == selfId) ? std::max(0.0, capS - prog) : capS;
            const double dropGain = cubeValue(myCubes, hpFrac) +
                                    cubeValue(myCubes + 1, hpFrac) +
                                    cubeValue(myCubes + 2, hpFrac) +
                                    230.0 * std::clamp(1.0 - hpFrac, 0.0, 1.0) +
                                    25.0 * std::clamp(1.0 - energy / maxEnergy, 0.0, 1.0);
            // Global-visibility beacon draws the lobby: only chase if already holding OR close with few foes alive!
            const double crowdRisk = (holder == selfId)
                ? 0.15
                : (0.85 + 0.22 * static_cast<double>(alive) + 0.04 * (moveT + waitLand));
            evaluate("drop", did, dpos, dropGain, waitLand + remCap, crowdRisk);
        }
    }

    if (!best) {
        lootID_ = -1;
        return std::nullopt;
    }
    const double maxLootTime = best->isRoof ? 12.0 : 30.0;
    if (best->id != lootID_) {
        lootID_ = best->id;
        lootSince_ = now_;
    } else if (now_ - lootSince_ > maxLootTime) {
        badLoot_[best->id] = now_ + 60.0;
        lootID_ = -1;
        roofFlying_ = false;
        return std::nullopt;
    }
    return best;
}

double Brain::homeCost(const Json& obs, const Json& me, double px, double py) const {
    const Vec3 pos = safeVec3(me, "pos");
    const auto& z = obs.contains("zone") && obs["zone"].is_object() ? obs["zone"] : Json::object();
    const Vec2 nc = safeVec2(z, "next_center");
    const double nr = numOr(z, "next_radius", 200.0);
    double crowd = 0.0;
    for (const auto& [id, t] : tracks_) {
        (void)id;
        if (now_ - t.seen < 20.0) {
            crowd += std::max(0.0, 80.0 - std::hypot(t.pos[0] - px, t.pos[1] - py)) / 80.0;
        }
    }
    // Layer 3 Map Memory: gently avoid sectors where strong foes were last seen 20s..50s ago
    for (const auto& [rid, re] : roster_) {
        (void)rid;
        const double age = now_ - re.lastSeen;
        if (re.dead || age < 20.0 || age > 50.0) continue;
        if (!re.hasLaser && !re.hasLightning && re.cubes < 4) continue;
        const double d = std::hypot(re.lastPos[0] - px, re.lastPos[1] - py);
        if (d < 90.0) {
            crowd += 0.55 * std::exp(-(age - 20.0) / 25.0) * (90.0 - d) / 90.0;
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
    const double heat = std::min(soundHeat(px, py), 2.0);
    const double wHeat = std::min(warzoneHeat(px, py), 2.5);
    const double trap = trapPenalty(px, py);
    const double canopy = arena_.canopyConcealment(px, py, arena_.walkRaster(px, py));
    // Layer 1 Map Memory: early-game gentle preference for unexplored low-exposure cover when missing gear
    const bool needGear = (intOr(me, "cubes", 0) < 3) && (now_ < 130.0);
    const double exploreBonus = needGear ? (8.0 * unvisitedScore(px, py)) : 0.0;
    return 120.0 * nav_.exposure(px, py) + 0.25 * std::hypot(px - pos[0], py - pos[1]) +
           40.0 * crowd + 15.0 * heat + 18.0 * wHeat + trap + 10.0 * edge - 6.0 * canopy - exploreBonus;
}

std::optional<Vec2> Brain::findHome(const Json& obs, const Json& me) {
    const auto& z = obs.contains("zone") && obs["zone"].is_object() ? obs["zone"] : Json::object();
    const Vec2 nc = safeVec2(z, "next_center");
    const double nr = numOr(z, "next_radius", 200.0);
    const bool out = !homeSet_ || std::hypot(homeXY_[0] - nc[0], homeXY_[1] - nc[1]) > nr * 0.8;
    if (out || now_ - homeAt_ > 15.0) {
        homeAt_ = now_;
        Vec2 best{};
        double bestC = kInf;
        std::uniform_real_distribution<double> u01(0.0, 1.0);
        for (int k = 0; k < 40; ++k) {
            const double a = u01(rng_) * kTau;
            const double r = std::sqrt(u01(rng_)) * nr * 0.72;
            const double px = nc[0] + std::cos(a) * r, py = nc[1] + std::sin(a) * r;
            if (!nav_.inBounds(px, py) || !nav_.reachable(px, py)) continue;
            const double c = homeCost(obs, me, px, py);
            if (c < bestC) {
                bestC = c;
                best = {px, py};
            }
        }
        // Strong hysteresis (-36) when already holding a valid low-exposure home inside next_center
        // so transient distant gunshots never kick us out of solid cover into open streets!
        if (std::isfinite(bestC) && (out || bestC < homeCost(obs, me, homeXY_[0], homeXY_[1]) - 36.0)) {
            homeXY_ = best;
            homeSet_ = true;
        }
    }
    if (!homeSet_) return std::nullopt;
    return homeXY_;
}

Vec2 Brain::laserDodgeDirection(const Json& obs, Vec3 pos) const {
    Vec2 direction{};
    if (const auto mit = obs.find("mechs"); mit != obs.end() && mit->is_array()) {
        for (const auto& m : *mit) {
            if (!m.is_object() || strOr(m, "kind") != "mech" || !boolOr(m, "charging", false)) continue;
            const Vec3 mp = safeVec3(m, "pos");
            const Vec2 maim = safeVec2(m, "aim");
            const double toMe = std::atan2(pos[1] - mp[1], pos[0] - mp[0]);
            if (std::abs(angDiff(maim[0], toMe)) > 0.16) continue;
            const double side = toMe + std::numbers::pi * 0.5 * strafe_;
            direction[0] += 1.5 * std::cos(side);
            direction[1] += 1.5 * std::sin(side);
        }
    }
    return direction;
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
                                     double v, double g, double minPitch, double maxPitch,
                                     bool allowHighArc) const {
    if (v <= 0.0 || g <= 0.0) return std::nullopt;
    auto solveArc = [&](double sign) -> std::optional<Vec2> {
        double t = 0.0, yaw = 0.0, pitch = 0.0;
        for (int k = 0; k < 3; ++k) {
            const double px = foot[0] + tv[0] * (kLat + t);
            const double py = foot[1] + tv[1] * (kLat + t);
            const double dx = std::hypot(px - tube[0], py - tube[1]);
            const double dz = foot[2] - tube[2];
            const double disc = v * v * v * v - g * (g * dx * dx + 2.0 * dz * v * v);
            if (disc < 0.0 || dx < 1.0) return std::nullopt;
            pitch = std::atan((v * v + sign * std::sqrt(disc)) / (g * dx));
            yaw = std::atan2(py - tube[1], px - tube[0]);
            const double c = std::cos(pitch);
            if (c < 0.05) return std::nullopt;
            t = dx / (v * c);
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
    };

    if (const auto low = solveArc(-1.0)) return low;
    if (allowHighArc) {
        return solveArc(1.0);
    }
    return std::nullopt;
}

Brain::FirePlan Brain::planDirectFire(const FireState& state, const Weapons& ws,
                                      const MechTrack* target, const Vec3& tp, const Vec3& tv,
                                      double tr, double th, bool movingToCover) const {
    const auto& [x, y, zf] = state.pos;
    const Vec3 muzzle{x + state.vel[0] * kLat, y + state.vel[1] * kLat, zf + state.muzzleHeight};
    const auto& curAim = state.aim;
    const double turnRate = state.turnRate, pitchRate = state.pitchRate;
    const double minPitch = state.minPitch, maxPitch = state.maxPitch;
    const double ownHeight = state.height, energy = state.energy;
    const int myCubes = state.cubes;
    const WeaponInfo* gun = getWeapon(ws, "gun");
    FirePlan result;
    Vec2& aim = result.aim;
    auto& fire = result.fire;
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

    const double tolY = std::atan2(tr * 0.75, std::max(dist, 1.0));
    const double tolP = std::atan2(th * 0.40, std::max(dist, 1.0));
    result.gunAim = gAim;
    result.hitscanAim = {hy, hpPitch};
    result.yawTolerance = tolY;
    result.pitchTolerance = tolP;
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
            if (!target || dist > w.range * 0.97) continue;
            // Don't start a fresh laser charge (0.5x move speed!) while actively running to cover
            if (movingToCover && w.charge <= 0.0) continue;
            const double full = w.fullCharge;
            // Always-on exact physics lethal early release (with cube + cook-off + rear-armor multipliers!)
            if (w.charge >= w.minCharge && w.charge < full - 0.051) {
                double dmg = w.damage * (w.minFrac + (1.0 - w.minFrac) * (w.charge / full)) *
                    (1.0 + cfgDmgPerCube_ * static_cast<double>(std::min(myCubes, cfgCubeStatCap_)));
                if (holdsKind(target->weapons, "mortar")) dmg *= (1.0 + w.cookoff);
                if (showsBack(*target, x, y)) dmg *= cfgRearMult_;
                if (dmg * 0.80 >= target->hp &&
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
            if (!target || w.cooldown > 0.25 || dist < 16.0 || dist > 200.0) continue;
            const Vec3 tube{muzzle[0], muzzle[1], zf + ownHeight + 0.5};
            const Vec3 foot{tp[0], tp[1], tp[2] - th * 0.5};
            if (const auto ma = mortarAim(tube, foot, tv, w.speed, w.gravity, minPitch, maxPitch, false)) {
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
    return result;
}

struct Brain::ProjectionContext {
    struct Threat {
        const MechTrack* mech;
        double damageRate;
        bool chasing;
    };

    FireState fire;
    Weapons weapons;
    const MechTrack* target = nullptr;
    std::vector<Threat> threats;
    std::vector<Projectile> projectiles;
    std::vector<Vec3> lightningInterceptors;
    std::array<ZoneCircle, kProjectionTicks + 1> circles{};
    double hp = 1400.0;
    double maxHp = 1400.0;
    double armor = 0.0;
    double maxEnergy = 100.0;
    double walk = 11.7;
    double boost = 19.5;
    double accel = 58.5;
    double airAccel = 12.0;
    double gravity = 22.0;
    double radius = 2.5;
    double stepUp = 1.2;
    double maxSlope = 50.0;
    double boostDrain = 17.0;
    double energyRegen = 18.0;
    double energyDelay = 0.6;
    double chargeMove = 0.5;
    double regenPerS = 45.0;
    double zoneDPS = 20.0;
    double backgroundDPS = 0.0;
    bool inZone = true;
    bool enteringZone = false;
    bool grounded = true;
    Vec2 dodge{};
    bool dodgeLaser = false;
};

struct Brain::ProjectedAction {
    ActionChoice first;
    std::optional<Vec2> waypoint;
    std::optional<Vec2> goal;
};

struct Brain::Projection {
    FireState fire;
    Weapons weapons;
    std::array<Vec3, 4> enemies{};
    std::array<double, 4> pursuitSide{};
    std::array<bool, 8> incomingHit{};
    std::array<double, kProjectionTicks + 1> pendingDamage{};
    MechTrack target;
    double hp = 0.0;
    double deathTime = kInf;
    double outsideTime = 0.0;
    double exposedTime = 0.0;
    double lastCombat = 0.0;
    double lastEnergyUse = 0.0;
    double stoppedOnWall = 0.0;
    bool reachedCover = false;
    bool settled = false;
};

Brain::ProjectionContext Brain::movementContext(const Json& me, const FireState& fire,
                                                const Weapons& weapons) const {
    ProjectionContext c;
    c.fire = fire;
    c.weapons = weapons;
    c.hp = numOr(me, "hp", 1400.0);
    c.maxHp = numOr(me, "max_hp", 1400.0);
    c.maxEnergy = numOr(me, "max_energy", 100.0);
    c.inZone = boolOr(me, "in_zone", true);
    c.grounded = boolOr(me, "grounded", true);
    const auto& st = me.contains("stats") && me["stats"].is_object() ? me["stats"] : Json::object();
    c.armor = numOr(st, "armor", 0.0);
    c.walk = std::max(1.0, numOr(st, "walk_speed", 11.7));
    c.boost = std::max(c.walk, numOr(st, "boost_speed", 19.5));
    c.accel = std::max(1.0, numOr(st, "ground_accel", 58.5));
    c.airAccel = std::max(1.0, numOr(st, "air_accel", 12.0));
    c.gravity = numOr(st, "gravity", 22.0);
    c.radius = numOr(st, "radius", cfgMechRadius_);
    c.stepUp = numOr(st, "step_up", 1.2);
    c.maxSlope = numOr(st, "max_slope_deg", 50.0);
    c.boostDrain = numOr(st, "boost_drain", 17.0);
    c.energyRegen = numOr(st, "energy_regen", cfgEnergyRegen_);
    c.energyDelay = numOr(st, "energy_regen_delay", 0.6);
    c.chargeMove = std::clamp(numOr(st, "charge_move_mult", 0.5), 0.1, 1.0);
    c.regenPerS = numOr(st, "regen_per_s", cfgRegenPerS_);
    return c;
}

Brain::ProjectionContext Brain::projectionContext(const Json& obs, const ProjectionContext& movement,
                                                  const ProjectileRisk& incoming, Vec2 laserDodge,
                                                  const MechTrack* target, const std::vector<MechTrack*>& seen,
                                                  const std::vector<Projectile>& projectiles, bool enteringZone) const {
    ProjectionContext c = movement;
    c.target = target;
    c.enteringZone = enteringZone;
    const auto& zone = obs.contains("zone") && obs["zone"].is_object() ? obs["zone"] : Json::object();
    c.zoneDPS = numOr(zone, "dps", 20.0);
    c.circles[0] = curZone(zone);
    const double shrinkStart = numOr(zone, "shrink_start", 0.0);
    const double shrinkEnd = numOr(zone, "shrink_end", 0.0);
    const auto next = safeVec2(zone, "next_center", {c.circles[0].x, c.circles[0].y});
    const double nextRadius = numOr(zone, "next_radius", c.circles[0].r);
    for (int tick = 1; tick <= kProjectionTicks; ++tick) {
        if (now_ >= shrinkStart && now_ < shrinkEnd) {
            const auto& current = c.circles[0];
            const double fraction = std::min(1.0, tick * kTick / (shrinkEnd - now_));
            c.circles[tick] = {std::lerp(current.x, next[0], fraction), std::lerp(current.y, next[1], fraction),
                               std::lerp(current.r, nextRadius, fraction)};
        } else {
            c.circles[tick] = zoneAt(zone, now_ + tick * kTick);
        }
    }
    c.dodgeLaser = std::hypot(laserDodge[0], laserDodge[1]) > 0.01;
    c.dodge = limitLength({incoming.side[0] + laserDodge[0], incoming.side[1] + laserDodge[1]}, 1.0);

    const FireState& fire = c.fire;
    const Weapons& weapons = c.weapons;
    for (const MechTrack* enemy : seen) {
        const double dist = distanceTo(fire.pos, {enemy->pos[0], enemy->pos[1]});
        const double attention = attentionWeight(*enemy, fire.pos[0], fire.pos[1]);
        const bool aiming = attention > 0.7;
        const double rate = enemyExpectedDPS(*enemy, weapons, c.armor, fire.energy, dist, aiming, false);
        const double towardMe = std::atan2(fire.pos[1] - enemy->pos[1], fire.pos[0] - enemy->pos[0]);
        const bool facingMe = std::abs(angDiff(enemy->aim[0], towardMe)) < 0.35;
        const bool chasing = now_ - enemy->hitMe < 4.5 ||
            (enemy->seen == now_ && facingMe && (dist < 65.0 || enemy->closing > 0.0));
        c.threats.push_back({enemy, rate * attention, chasing});
    }
    std::stable_sort(c.threats.begin(), c.threats.end(), [&](const auto& a, const auto& b) {
        if ((a.mech == target) != (b.mech == target)) return a.mech == target;
        return a.damageRate > b.damageRate;
    });
    for (std::size_t i = 4; i < c.threats.size(); ++i) {
        if (c.threats[i].mech->los) c.backgroundDPS += c.threats[i].damageRate;
    }
    if (c.threats.size() > 4) c.threats.resize(4);

    constexpr std::size_t projectionLimit = 8;
    c.projectiles.assign(projectiles.begin(), projectiles.begin() + std::min(projectiles.size(), projectionLimit));
    if (getWeapon(weapons, "lightning") && target) {
        for (const char* field : {"mechs", "crates"}) {
            if (const auto it = obs.find(field); it != obs.end() && it->is_array()) {
                for (const auto& item : *it) {
                    if (intOr(item, "id", -1) == target->id) continue;
                    const Vec3 pos = safeVec3(item, "pos");
                    if (distanceTo(fire.pos, {pos[0], pos[1]}) <= 2.0 * getWeapon(weapons, "lightning")->range) {
                        c.lightningInterceptors.push_back(pos);
                    }
                }
            }
        }
    }
    return c;
}

bool Brain::projectMove(const ProjectionContext& c, const Vec3& from, Vec3& to) const {
    if (!nav_.inBounds(to[0], to[1])) return false;
    to[2] = arena_.surface(to[0], to[1], from[2] + c.stepUp + 1e-6);
    return arena_.walkSegment(from, to, c.radius, c.fire.height, c.stepUp, c.maxSlope);
}

bool Brain::advanceGroundMotion(const ProjectionContext& c, FireState& state, Vec2 move, double speed) const {
    const Vec2 change = limitLength({move[0] * speed - state.vel[0], move[1] * speed - state.vel[1]}, c.accel * kTick);
    state.vel[0] += change[0];
    state.vel[1] += change[1];
    const Vec3 start = state.pos;
    Vec3 next{start[0] + state.vel[0] * kTick, start[1] + state.vel[1] * kTick, start[2]};
    if (next == start || projectMove(c, start, next)) {
        state.pos = next;
        return true;
    }
    Vec3 slideX{next[0], start[1], start[2]};
    Vec3 slideY{start[0], next[1], start[2]};
    const bool canSlideX = slideX[0] != start[0] && projectMove(c, start, slideX);
    const bool canSlideY = slideY[1] != start[1] && projectMove(c, start, slideY);
    if (canSlideX && (!canSlideY || std::abs(state.vel[0]) >= std::abs(state.vel[1]))) {
        state.pos = slideX;
        state.vel[1] = 0.0;
        return true;
    }
    if (canSlideY) {
        state.pos = slideY;
        state.vel[0] = 0.0;
        return true;
    }
    state.vel[0] = state.vel[1] = 0.0;
    return false;
}

std::vector<Brain::Projectile> Brain::nearbyProjectiles(const Json& obs, const ProjectionContext& c) const {
    const auto it = obs.find("projectiles");
    if (it == obs.end() || !it->is_array() || it->empty()) return {};
    constexpr double horizon = kProjectileTicks * kTick;
    const double reach = c.radius + horizon * std::max(c.boost, std::hypot(c.fire.vel[0], c.fire.vel[1]));
    const auto prop = config_.find("projectile");
    const double gunRadius = prop != config_.end() ? numOr(*prop, "gun_radius", 0.3) : 0.3;
    const double npcRadius = prop != config_.end() ? numOr(*prop, "npc_radius", 0.3) : 0.3;
    std::vector<Projectile> rounds;
    for (const auto& item : *it) {
        const std::string kind = strOr(item, "kind");
        if (kind != "gun" && kind != "npc") continue;
        const Vec3 p = safeVec3(item, "pos");
        const Vec3 v = safeVec3(item, "vel");
        const double speed2 = v[0] * v[0] + v[1] * v[1];
        const double dx = c.fire.pos[0] - p[0], dy = c.fire.pos[1] - p[1];
        const double t = speed2 > 1e-9 ? std::clamp((dx * v[0] + dy * v[1]) / speed2, 0.0, horizon) : 0.0;
        const double radius = kind == "npc" ? npcRadius : gunRadius;
        if (std::hypot(dx - t * v[0], dy - t * v[1]) <= reach + radius) {
            rounds.push_back({p, v, radius, kind == "npc" ? 25.0 : 45.0});
        }
    }
    return rounds;
}

std::optional<double> Brain::projectileContact(const Projectile& p, double stepStart, const Vec3& before,
                                                const Vec3& after, double radius, double height) const {
    // Move into the mech's frame, then intersect the horizontal disk and vertical span.
    const double x = p.pos[0] + p.vel[0] * stepStart - before[0];
    const double y = p.pos[1] + p.vel[1] * stepStart - before[1];
    const double z = p.pos[2] + p.vel[2] * stepStart - before[2];
    const double dx = p.vel[0] * kTick - (after[0] - before[0]);
    const double dy = p.vel[1] * kTick - (after[1] - before[1]);
    const double dz = p.vel[2] * kTick - (after[2] - before[2]);
    const double reach = radius + p.radius;
    const double a = dx * dx + dy * dy;
    const double b = x * dx + y * dy;
    const double c = x * x + y * y - reach * reach;
    double low = 0.0, high = 1.0;
    if (a > 1e-9) {
        const double disc = b * b - a * c;
        if (disc < 0.0) return std::nullopt;
        const double root = std::sqrt(disc);
        low = std::max(low, (-b - root) / a);
        high = std::min(high, (-b + root) / a);
    } else if (c > 0.0) {
        return std::nullopt;
    }
    if (std::abs(dz) > 1e-9) {
        const double bottom = (-p.radius - z) / dz;
        const double top = (height + p.radius - z) / dz;
        low = std::max(low, std::min(bottom, top));
        high = std::min(high, std::max(bottom, top));
    } else if (z < -p.radius || z > height + p.radius) {
        return std::nullopt;
    }
    if (low > high) return std::nullopt;
    const double time = stepStart + low * kTick;
    const Vec3 impact{p.pos[0] + p.vel[0] * time, p.pos[1] + p.vel[1] * time, p.pos[2] + p.vel[2] * time};
    if (arena_.blocksLine(p.pos, impact)) return std::nullopt;
    return time;
}

Brain::ProjectileRisk Brain::forecastProjectileRisk(const ProjectionContext& c, ActionChoice action,
                                                    const std::vector<Projectile>& rounds) const {
    ProjectileRisk outcome;
    if (rounds.empty()) return outcome;
    FireState state = c.fire;
    const WeaponInfo* laser = getWeapon(c.weapons, "laser");
    const double chargeFactor = laser && laser->charge > 0.0 ? c.chargeMove : 1.0;
    std::vector<std::uint8_t> hit(rounds.size());
    for (int tick = 0; tick < kProjectileTicks; ++tick) {
        const Vec3 start = state.pos;
        const bool boosting = c.grounded && action.boost && state.energy > 0.1 && action.move != Vec2{};
        const double speed = (boosting ? c.boost : c.walk) * chargeFactor;
        if (c.grounded) {
            advanceGroundMotion(c, state, action.move, speed);
            if (boosting) state.energy -= c.boostDrain * kTick;
        } else {
            const Vec2 change = limitLength({action.move[0] * speed - state.vel[0],
                                            action.move[1] * speed - state.vel[1]}, c.airAccel * kTick);
            state.vel[0] += change[0];
            state.vel[1] += change[1];
            state.vel[2] -= c.gravity * kTick;
            for (int axis = 0; axis < 3; ++axis) state.pos[axis] += state.vel[axis] * kTick;
            const double ground = arena_.surface(state.pos[0], state.pos[1], start[2]);
            if (state.pos[2] < ground) {
                state.pos[2] = ground;
                state.vel[2] = 0.0;
            }
        }
        for (std::size_t i = 0; i < rounds.size(); ++i) {
            if (hit[i]) continue;
            const auto& p = rounds[i];
            const auto time = projectileContact(p, tick * kTick, start, state.pos, c.radius, state.height);
            if (!time) continue;
            hit[i] = true;
            outcome.damage += p.damage;
            if (*time >= outcome.firstHit) continue;
            outcome.firstHit = *time;
            outcome.bearing = std::atan2(p.pos[1] - state.pos[1], p.pos[0] - state.pos[0]);
            const double fraction = (*time - tick * kTick) / kTick;
            const double x = p.pos[0] + p.vel[0] * *time - std::lerp(start[0], state.pos[0], fraction);
            const double y = p.pos[1] + p.vel[1] * *time - std::lerp(start[1], state.pos[1], fraction);
            const double length = std::hypot(p.vel[0], p.vel[1]);
            Vec2 side = length > 1e-9 ? Vec2{-p.vel[1] / length, p.vel[0] / length} : Vec2{1.0, 0.0};
            if (side[0] * x + side[1] * y > 0.0) side = {-side[0], -side[1]};
            outcome.side = side;
        }
    }
    return outcome;
}

Brain::ActionChoice Brain::dodgeProjectiles(const ProjectionContext& movement, ActionChoice fallback,
                                            const std::vector<Projectile>& projectiles) const {
    const auto incoming = forecastProjectileRisk(movement, fallback, projectiles);
    if (incoming.damage <= 0.0) return fallback;
    ActionChoice best = fallback;
    double bestDamage = incoming.damage;
    const Vec2 side = incoming.side;
    const Vec2 other{-side[0], -side[1]};
    for (bool boost : {fallback.boost, !fallback.boost}) {
        if (boost && (!movement.grounded || movement.fire.energy <= 0.1)) continue;
        for (const Vec2& direction : {limitLength({0.4 * fallback.move[0] + 0.9 * side[0],
                                                   0.4 * fallback.move[1] + 0.9 * side[1]}, 1.0),
                                      limitLength({0.4 * fallback.move[0] + 0.9 * other[0],
                                                   0.4 * fallback.move[1] + 0.9 * other[1]}, 1.0),
                                      side, other, Vec2{}}) {
            const ActionChoice action{direction, boost};
            const auto risk = forecastProjectileRisk(movement, action, projectiles);
            if (risk.damage < bestDamage) {
                bestDamage = risk.damage;
                best = action;
                if (bestDamage <= 0.0) return best;
            }
        }
    }
    return best;
}

Vec2 Brain::combatRoundDodge(const ProjectionContext& c, const std::vector<Projectile>& rounds) const {
    // While dueling, leave a decision's worth of lateral clearance for follow-up fire.
    const Vec3& feet = c.fire.pos;
    const double reach = c.radius + c.walk * kDecision;
    Vec2 result{};
    for (const auto& p : rounds) {
        const double x = feet[0] - p.pos[0], y = feet[1] - p.pos[1];
        const double z = feet[2] + c.fire.height * 0.56 - p.pos[2];
        const double horizontal2 = p.vel[0] * p.vel[0] + p.vel[1] * p.vel[1];
        if (horizontal2 < 1.0) continue;
        const double speed2 = horizontal2 + p.vel[2] * p.vel[2];
        const double t = (x * p.vel[0] + y * p.vel[1] + z * p.vel[2]) / speed2;
        if (t <= 0.0 || t > kProjectileTicks * kTick) continue;
        const double dx = p.vel[0] * t - x, dy = p.vel[1] * t - y;
        if (std::hypot(dx, dy) >= reach) continue;
        const Vec3 intercept{p.pos[0] + p.vel[0] * t, p.pos[1] + p.vel[1] * t, p.pos[2] + p.vel[2] * t};
        if (intercept[2] < feet[2] - p.radius || intercept[2] > feet[2] + c.fire.height + p.radius ||
            arena_.blocksLine(p.pos, intercept)) continue;
        const double speed = std::sqrt(horizontal2);
        const Vec2 side{-p.vel[1] / speed, p.vel[0] / speed};
        const double sign = side[0] * dx + side[1] * dy > 0.0 ? -1.0 : 1.0;
        result[0] += sign * side[0];
        result[1] += sign * side[1];
    }
    return result;
}

bool Brain::canFinishWithinProjection(const FireState& fs, const Weapons& weapons,
                                       const MechTrack& target) const {
    // An optimistic bound limits this decision to genuine finish opportunities. Ordinary
    // retreats keep their existing route when even every available shot would fall short.
    constexpr double horizon = kProjectionDecisions * kDecision;
    double damage = 0.0;
    for (const auto& slot : weapons) {
        if (!slot) continue;
        const auto& w = *slot;
        double ready = w.cooldown;
        double shotDamage = w.damage;
        if (w.kind == "gun") {
            const double reload = w.ammo <= 0.0 && w.reload <= 0.0 ? w.reloadS : w.reload;
            ready = std::max(ready, reload) + target.dist / std::max(1.0, w.speed);
        } else if (w.kind == "laser") {
            ready = std::max(ready, laserShortUntil_ - now_) + std::max(0.0, w.minCharge - w.charge);
            if (holdsKind(target.weapons, "mortar")) shotDamage *= 1.0 + w.cookoff;
        } else if (w.kind != "lightning") {
            continue;
        }
        if (ready > horizon) continue;
        double shots = 1.0 + std::floor((horizon - ready) / w.fireInterval);
        if (w.kind == "lightning" && w.energyCost > 0.0) {
            shots = std::min(shots, std::floor((fs.energy + cfgEnergyRegen_ * horizon) / w.energyCost));
        }
        damage += shots * shotDamage;
    }
    const double cubeMult = 1.0 + cfgDmgPerCube_ * std::min(fs.cubes, cfgCubeStatCap_);
    return damage * cubeMult * cfgRearMult_ >= target.hp;
}

void Brain::projectAction(const ProjectionContext& c, const ProjectedAction& action,
                          Projection& s, bool pursuit, int decision) const {
    if (s.hp <= 0.0) return;
    const double elapsed = decision * kDecision;
    auto& fs = s.fire;
    const WeaponInfo* laser = getWeapon(s.weapons, "laser");
    const double chargeFactor = laser && laser->charge > 0.0 ? c.chargeMove : 1.0;
    Vec2 move = action.first.move;
    if (decision > 0 && action.waypoint) {
        Vec2 waypoint = *action.waypoint;
        if (action.goal && distanceTo(fs.pos, waypoint) <= c.walk * kDecision) waypoint = *action.goal;
        const double distance = distanceTo(fs.pos, waypoint);
        const double speed = (action.first.boost ? c.boost : c.walk) * chargeFactor;
        move = toward(fs.pos, waypoint);
        const double fraction = std::clamp(distance / (speed * kDecision), 0.0, 1.0);
        move[0] *= fraction;
        move[1] *= fraction;
    }

    double inDPS = c.backgroundDPS;
    double fromTarget = 0.0;
    bool exposed = c.backgroundDPS > 0.0;
    bool targetVisible = false;
    const Vec3 body{fs.pos[0], fs.pos[1], fs.pos[2] + fs.height * 0.6};
    for (std::size_t i = 0; i < c.threats.size(); ++i) {
        const auto& track = c.threats[i];
        const auto& enemy = *track.mech;
        const bool isTarget = track.mech == c.target;
        if (isTarget && s.target.hp <= 0.0) continue;
        Vec3& pos = s.enemies[i];
        double vx = enemy.vel[0], vy = enemy.vel[1];
        const double distBefore = distanceTo(fs.pos, {pos[0], pos[1]});
        if (pursuit && track.chasing) {
            const Vec2 toMe = toward(pos, {fs.pos[0], fs.pos[1]});
            const double clearance = std::max(0.0, distBefore - c.radius - cfgMechRadius_ * scaleOf(enemy.cubes));
            const double speed = std::min(std::max(11.7, std::hypot(vx, vy)), clearance / kDecision);
            vx = toMe[0] * speed;
            vy = toMe[1] * speed;
        }
        if (decision > 0 && (vx != 0.0 || vy != 0.0)) {
            auto step = [&](double dx, double dy) {
                const double nx = pos[0] + dx * kDecision, ny = pos[1] + dy * kDecision;
                if (!nav_.inBounds(nx, ny) || !nav_.free(nx, ny) || !nav_.lineWalkable(pos[0], pos[1], nx, ny)) {
                    return false;
                }
                pos = {nx, ny, arena_.surface(nx, ny, pos[2] + c.stepUp + 1e-6)};
                return true;
            };
            if (step(vx, vy)) {
                s.pursuitSide[i] = 0.0;
            } else if (pursuit && track.chasing) {
                // Continue around the same side of a corner until a direct approach opens.
                const double side = s.pursuitSide[i] == 0.0 ? 1.0 : s.pursuitSide[i];
                const double heading = std::atan2(vy, vx), speed = std::hypot(vx, vy);
                for (double turn : {side, -side, 2.0 * side, -2.0 * side, 3.0 * side, -3.0 * side}) {
                    const double a = heading + turn * std::numbers::pi / 4.0;
                    const double dx = speed * std::cos(a), dy = speed * std::sin(a);
                    if (step(dx, dy)) {
                        vx = dx;
                        vy = dy;
                        s.pursuitSide[i] = std::copysign(1.0, turn);
                        break;
                    }
                }
            }
        }
        const double dist = distanceTo(fs.pos, {pos[0], pos[1]});
        const Vec3 eye{pos[0], pos[1], pos[2] + cfgMechEye_ * scaleOf(enemy.cubes)};
        const bool blocked = arena_.blocksLine(eye, body);
        if (isTarget) {
            s.target.pos = pos;
            s.target.vel = {vx, vy, enemy.vel[2]};
            if (pursuit && track.chasing && decision > 0) {
                s.target.aim[0] = std::atan2(fs.pos[1] - pos[1], fs.pos[0] - pos[0]);
            }
            targetVisible = !blocked || (decision == 0 && enemy.los);
        }
        if (blocked) continue;
        const double bearing = std::atan2(fs.pos[1] - pos[1], fs.pos[0] - pos[0]);
        const double off = std::abs(angDiff(enemy.aim[0], bearing));
        const bool facing = off < 0.35 || (pursuit && track.chasing);
        const double attention = track.chasing || dist < 42.0 ? 1.0 : (facing ? 0.95 : (off < 0.85 ? 0.55 : 0.25));
        double rate = enemyExpectedDPS(enemy, s.weapons, c.armor, fs.energy, dist, facing, false) * attention;
        if (enemy.charging && facing && elapsed < std::max(0.0, 0.75 - kDecision)) rate += 315.0;
        if (dist > 48.0 && arena_.blocksVisionTree(eye, body)) rate *= 0.15;
        const double toEnemy = std::atan2(pos[1] - fs.pos[1], pos[0] - fs.pos[0]);
        if (std::abs(angDiff(fs.aim[0], toEnemy)) > radians(cfgFrontArcDeg_ * 0.5)) rate *= cfgRearMult_;
        inDPS += rate;
        if (isTarget) fromTarget = rate;
        exposed = exposed || rate > 0.0;
    }

    if (decision > 0 && !exposed) s.reachedCover = true;
    const bool quiet = action.first.waitForCover && !s.reachedCover;
    FirePlan shot;
    shot.aim = fs.aim;
    double targetDistance = 0.0;
    if (targetVisible) {
        const double scale = scaleOf(s.target.cubes);
        const double height = cfgMechHeight_ * scale;
        Vec3 point = s.target.pos;
        point[2] += height * 0.5;
        const Vec3 muzzle{fs.pos[0] + fs.vel[0] * kLat, fs.pos[1] + fs.vel[1] * kLat, fs.pos[2] + fs.muzzleHeight};
        if (arena_.blocksLine(muzzle, point)) point[2] = s.target.pos[2] + height * 0.85;
        targetVisible = !arena_.blocksLine(muzzle, point);
        targetDistance = distanceTo(fs.pos, {point[0], point[1]});
        if (targetVisible) {
            const bool movingToCover = !c.enteringZone && !action.first.stayToFight &&
                (action.goal || action.first.boost) && (move[0] != 0.0 || move[1] != 0.0);
            shot = planDirectFire(fs, s.weapons, &s.target, point, s.target.vel, cfgMechRadius_ * scale,
                                  height, movingToCover);
        }
    }
    if (quiet) shot.fire.fill(false);

    for (int substep = 1; substep <= 2; ++substep) {
        const int tick = 2 * decision + substep;
        const double time = tick * kTick;
        fs.aim[0] += std::clamp(angDiff(shot.aim[0], fs.aim[0]), -fs.turnRate * kTick, fs.turnRate * kTick);
        fs.aim[1] = std::clamp(fs.aim[1] + std::clamp(shot.aim[1] - fs.aim[1], -fs.pitchRate * kTick, fs.pitchRate * kTick),
                              fs.minPitch, fs.maxPitch);
        const bool boosting = action.first.boost && fs.energy > 0.1 && (move[0] != 0.0 || move[1] != 0.0);
        const double speed = (boosting ? c.boost : c.walk) *
                             ((laser && laser->charge > 0.0) ? c.chargeMove : 1.0);
        const Vec3 before = fs.pos;
        if (!advanceGroundMotion(c, fs, move, speed)) s.stoppedOnWall += kTick;
        if (boosting) {
            fs.energy = std::max(0.0, fs.energy - c.boostDrain * kTick);
            s.lastEnergyUse = now_ + time;
        } else if (now_ + time - s.lastEnergyUse >= c.energyDelay) {
            fs.energy = std::min(c.maxEnergy, fs.energy + c.energyRegen * kTick);
        }

        const auto& circle = c.circles[tick];
        const bool outside = distanceTo(fs.pos, {circle.x, circle.y}) > circle.r;
        if (outside) s.outsideTime += kTick;
        if (exposed) s.exposedTime += kTick;
        double damage = (inDPS + (outside ? c.zoneDPS : 0.0)) * kTick;
        for (std::size_t i = 0; i < c.projectiles.size(); ++i) {
            if (s.incomingHit[i]) continue;
            const auto& p = c.projectiles[i];
            if (!projectileContact(p, time - kTick, before, fs.pos, c.radius, fs.height)) continue;
            damage += p.damage * std::clamp(1.0 - c.armor, 0.25, 1.0);
            s.incomingHit[i] = true;
        }
        s.hp -= damage;
        if (damage > 0.0) s.lastCombat = now_ + time;
        if (s.hp <= 0.0) {
            s.deathTime = time;
            s.settled = true;
            return;
        }

        const double cubeMult = 1.0 + cfgDmgPerCube_ * std::min(fs.cubes, cfgCubeStatCap_);
        const double rear = showsBack(s.target, fs.pos[0], fs.pos[1]) ? cfgRearMult_ : 1.0;
        for (std::size_t slot = 0; slot < s.weapons.size(); ++slot) {
            if (!s.weapons[slot]) continue;
            auto& w = *s.weapons[slot];
            w.cooldown = std::max(0.0, w.cooldown - kTick);
            const bool holding = shot.fire[slot] && s.target.hp > 0.0;
            const auto aligned = [&](Vec2 aim, double yawTol) {
                return std::abs(angDiff(fs.aim[0], aim[0])) < yawTol &&
                       std::abs(fs.aim[1] - aim[1]) < shot.pitchTolerance;
            };
            if (w.kind == "gun") {
                if (w.reload > 0.0) {
                    w.reload = std::max(0.0, w.reload - kTick);
                    if (w.reload <= 0.0) w.ammo = w.mag;
                }
                if (!holding || w.cooldown > 0.0 || w.reload > 0.0 || w.ammo <= 0.0) continue;
                --w.ammo;
                w.cooldown = w.fireInterval;
                if (w.ammo <= 0.0) w.reload = w.reloadS;
                if (targetVisible && aligned(shot.gunAim, shot.yawTolerance)) {
                    const double flight = targetDistance / std::max(1.0, w.speed);
                    const int impact = tick + static_cast<int>(std::ceil(flight / kTick));
                    if (impact <= kProjectionTicks) {
                        s.pendingDamage[impact] += w.damage * cubeMult * rear * gunHitProb(targetDistance);
                    }
                }
            } else if (w.kind == "laser") {
                if (now_ + time < laserShortUntil_) {
                    w.charge = 0.0;
                } else if (w.charge <= 0.0) {
                    if (holding && w.cooldown <= 0.0) w.charge = kTick;
                } else {
                    w.charge += kTick;
                    if ((!holding && w.charge >= w.minCharge) || w.charge >= w.fullCharge + w.holdS) {
                        const double fraction = w.minFrac + (1.0 - w.minFrac) * std::min(1.0, w.charge / w.fullCharge);
                        if (targetVisible && targetDistance <= w.range && aligned(shot.hitscanAim, shot.yawTolerance * 0.9)) {
                            const double cookoff = holdsKind(s.target.weapons, "mortar") ? 1.0 + w.cookoff : 1.0;
                            s.pendingDamage[tick] += w.damage * fraction * cubeMult * cookoff * rear * 0.8;
                        }
                        w.charge = 0.0;
                        w.cooldown = w.fireInterval;
                    }
                }
            } else if (w.kind == "lightning" && holding && w.cooldown <= 0.0 && fs.energy >= w.energyCost) {
                fs.energy -= w.energyCost;
                s.lastEnergyUse = now_ + time;
                w.cooldown = w.fireInterval;
                const double cone = radians(w.coneDeg) * 0.5;
                if (!targetVisible || targetDistance >= w.range ||
                    std::abs(angDiff(shot.hitscanAim[0], fs.aim[0])) >= cone) continue;
                const bool intercepted = std::any_of(c.lightningInterceptors.begin(), c.lightningInterceptors.end(),
                    [&](const Vec3& point) {
                        const double yaw = std::atan2(point[1] - fs.pos[1], point[0] - fs.pos[0]);
                        return distanceTo(fs.pos, {point[0], point[1]}) < targetDistance &&
                               std::abs(angDiff(yaw, fs.aim[0])) < cone &&
                               !arena_.blocksLine({fs.pos[0], fs.pos[1], fs.pos[2] + fs.muzzleHeight},
                                                  {point[0], point[1], point[2] + 1.0});
                    });
                if (!intercepted) s.pendingDamage[tick] += w.damage * cubeMult * rear;
            } else if (w.kind == "mortar" && holding && w.cooldown <= 0.0) {
                w.cooldown = w.fireInterval;
            }
        }
        if (s.pendingDamage[tick] > 0.0 && s.target.hp > 0.0) {
            s.target.hp -= s.pendingDamage[tick];
            s.lastCombat = now_ + time;
            if (s.target.hp <= 0.0) {
                targetVisible = false;
                inDPS = std::max(0.0, inDPS - fromTarget);
            }
        }
        if (!outside && now_ + time - s.lastCombat >= cfgRegenDelay_) {
            s.hp = std::min(c.maxHp, s.hp + c.regenPerS * kTick);
        }
    }
    const auto& circle = c.circles[2 * decision + 2];
    const bool inZone = distanceTo(fs.pos, {circle.x, circle.y}) <= circle.r;
    const bool arrived = action.goal && distanceTo(fs.pos, *action.goal) < c.walk * kDecision;
    if (c.enteringZone) {
        s.settled = inZone && (arrived || (!c.inZone && std::hypot(fs.vel[0], fs.vel[1]) <= c.accel * kDecision));
    } else {
        s.settled = (c.target && s.target.hp <= 0.0) || (arrived && !exposed);
    }
}

Brain::ActionChoice Brain::compareActions(const ProjectionContext& c, const std::vector<ProjectedAction>& actions,
                                           bool enteringZone) const {
    if (actions.size() <= 1) return actions.front().first;
    Projection initial;
    initial.fire = c.fire;
    initial.weapons = c.weapons;
    initial.hp = c.hp;
    initial.lastCombat = lastCombatAt_;
    initial.lastEnergyUse = c.fire.energy >= c.maxEnergy ? -kInf : now_;
    if (c.target) initial.target = *c.target;
    for (std::size_t i = 0; i < c.threats.size(); ++i) initial.enemies[i] = c.threats[i].mech->pos;

    std::vector<std::array<Projection, 2>> predictions(actions.size(), {initial, initial});
    const bool hasPursuit = std::any_of(c.threats.begin(), c.threats.end(), [](const auto& t) { return t.chasing; });
    // Evaluate every candidate over the same time. A candidate that reaches cover first still
    // has to survive there while the other candidate finishes its approach.
    for (int decision = 0; decision < kProjectionDecisions; ++decision) {
        bool settled = true;
        for (std::size_t i = 0; i < actions.size(); ++i) {
            for (std::size_t scenario = 0; scenario < (hasPursuit ? 2U : 1U); ++scenario) {
                auto& prediction = predictions[i][scenario];
                projectAction(c, actions[i], prediction, scenario != 0, decision);
                settled = settled && prediction.settled;
            }
        }
        if (settled && decision > 0) break;
    }

    struct Score {
        int deaths;
        double firstDeath;
        double hp;
        double primaryExposure;
        double progress;
        double blockedTime;
        double energy;
    };
    auto summarize = [&](std::size_t i) -> Score {
        const auto& a = predictions[i][0];
        const auto& b = predictions[i][hasPursuit ? 1 : 0];
        const auto& goal = actions[i].goal;
        return {
            static_cast<int>(a.hp <= 0.0) + static_cast<int>(b.hp <= 0.0),
            std::min(a.deathTime, b.deathTime),
            std::min(a.hp, b.hp),
            enteringZone ? std::max(a.outsideTime, b.outsideTime) : std::max(a.exposedTime, b.exposedTime),
            goal ? -std::max(distanceTo(a.fire.pos, *goal), distanceTo(b.fire.pos, *goal)) : 0.0,
            std::max(a.stoppedOnWall, b.stoppedOnWall),
            std::min(a.fire.energy, b.fire.energy),
        };
    };
    std::vector<Score> scores;
    scores.reserve(actions.size());
    int fewestDeaths = 2;
    for (std::size_t i = 0; i < actions.size(); ++i) {
        scores.push_back(summarize(i));
        fewestDeaths = std::min(fewestDeaths, scores.back().deaths);
    }
    double latestDeath = 0.0;
    for (const auto& score : scores) {
        if (score.deaths == fewestDeaths) latestDeath = std::max(latestDeath, score.firstDeath);
    }
    double bestHP = -kInf;
    for (const auto& score : scores) {
        if (score.deaths == fewestDeaths && score.firstDeath == latestDeath) bestHP = std::max(bestHP, score.hp);
    }
    // Sub-tick health estimates are too close to distinguish. Use the actual exposure and
    // progress in that case, before spending energy that won't make the route safer.
    const double hpResolution = std::max(1.0, c.regenPerS * kTick);
    std::size_t best = 0;
    bool found = false;
    for (std::size_t i = 0; i < scores.size(); ++i) {
        const auto& a = scores[i];
        if (a.deaths != fewestDeaths || a.firstDeath != latestDeath || a.hp < bestHP - hpResolution) continue;
        const auto& b = scores[best];
        if (!found || a.primaryExposure < b.primaryExposure - 1e-6 ||
            (std::abs(a.primaryExposure - b.primaryExposure) < 1e-6 &&
             (a.blockedTime < b.blockedTime - 1e-6 ||
              (std::abs(a.blockedTime - b.blockedTime) < 1e-6 &&
               (a.progress > b.progress + 0.05 ||
                (std::abs(a.progress - b.progress) <= 0.05 && a.energy > b.energy + 0.05)))))) {
            best = i;
            found = true;
        }
    }
    ActionChoice selected = actions[best].first;
    if (!enteringZone && !selected.stayToFight && !selected.waitForCover && c.target) {
        const auto& a = predictions[best][0];
        const auto& b = predictions[best][hasPursuit ? 1 : 0];
        selected.finishesWhileMoving = a.target.hp <= 0.0 && b.target.hp <= 0.0;
    }
    return selected;
}

Brain::ActionChoice Brain::chooseFinishAction(const ProjectionContext& c, ActionChoice fallback,
                                               std::optional<Vec2> cover, std::optional<ActionChoice> baselineFight) {
    const Vec3& start = c.fire.pos;
    std::optional<Vec2> waypoint;
    const Vec2 route = limitLength(fallback.move, 1.0);
    if (route[0] != 0.0 || route[1] != 0.0) {
        const double leg = cover ? std::min(5.0, distanceTo(start, *cover)) : 5.0;
        waypoint = Vec2{start[0] + route[0] * leg, start[1] + route[1] * leg};
    }
    std::vector<ProjectedAction> actions;
    if (baselineFight) actions.push_back({*baselineFight, std::nullopt, cover});
    actions.push_back({fallback, waypoint, cover});
    if (waypoint) {
        if (fallback.boost) actions.push_back({{fallback.move, false}, waypoint, cover});
        else if (c.fire.energy > 0.1) actions.push_back({{fallback.move, true}, waypoint, cover});
        if (cover) actions.push_back({{fallback.move, fallback.boost, true}, waypoint, cover});
    }
    actions.push_back({{{}, false, false, true}, std::nullopt, cover});
    if (!cover && !c.threats.empty()) {
        const auto mostDangerous = std::max_element(c.threats.begin(), c.threats.end(), [](const auto& a, const auto& b) {
            return a.damageRate < b.damageRate;
        });
        const auto& enemy = mostDangerous->mech->pos;
        const Vec2 away = toward(enemy, {start[0], start[1]});
        for (double angle : {0.0, -std::numbers::pi / 4.0, std::numbers::pi / 4.0}) {
            const Vec2 direction{away[0] * std::cos(angle) - away[1] * std::sin(angle),
                                 away[0] * std::sin(angle) + away[1] * std::cos(angle)};
            Vec3 next{start[0] + direction[0] * c.walk * kDecision,
                      start[1] + direction[1] * c.walk * kDecision, start[2]};
            if (next == start || !projectMove(c, start, next)) continue;
            actions.push_back({{direction, false}, std::nullopt, std::nullopt});
            if (c.fire.energy > 0.1) actions.push_back({{direction, true}, std::nullopt, std::nullopt});
        }
    }
    if (c.dodge[0] != 0.0 || c.dodge[1] != 0.0) {
        const Vec2 mv = limitLength({fallback.move[0] * 0.4 + c.dodge[0] * 0.9,
                                     fallback.move[1] * 0.4 + c.dodge[1] * 0.9}, 1.0);
        actions.push_back({{mv, fallback.boost || (c.dodgeLaser && c.fire.energy > 0.1)}, waypoint, cover});
    }
    return compareActions(c, actions, false);
}

Brain::ActionChoice Brain::chooseZoneAction(const ProjectionContext& c, ActionChoice fallback, Vec2 goal,
                                            bool followDetour) {
    const Vec3& start = c.fire.pos;
    const auto& circle = c.circles[2];
    const Vec2 center{circle.x, circle.y};
    std::optional<Vec2> entry;
    if (!c.inZone && distanceTo(start, center) - circle.r <= c.boost * 0.5) {
        const double bearing = std::atan2(start[1] - circle.y, start[0] - circle.x);
        const double radius = std::max(0.0, circle.r - std::min(1.0, circle.r * 0.5));
        double bestDistance = kInf;
        auto consider = [&](Vec2 point) {
            const double distance = distanceTo(start, point);
            if (distance <= 1e-6 || distance >= bestDistance || distance > c.boost * 0.5) return;
            Vec3 end{point[0], point[1], start[2]};
            if (projectMove(c, start, end)) {
                bestDistance = distance;
                entry = point;
            }
        };
        consider(center);
        if (radius > 0.0) {
            for (double ring : {radius, radius * 0.5, circle.r - 1e-3}) {
                if (ring <= 0.0) continue;
                for (int k = 0; k < 8; ++k) {
                    const double angle = bearing + k * kTau / 8.0;
                    consider({circle.x + std::cos(angle) * ring, circle.y + std::sin(angle) * ring});
                }
            }
        }
    }

    std::optional<Vec2> waypoint;
    const Vec2 route = limitLength(fallback.move, 1.0);
    if (route[0] != 0.0 || route[1] != 0.0) {
        const double leg = std::min(5.0, distanceTo(start, goal));
        waypoint = followDetour ? goal : Vec2{start[0] + route[0] * leg, start[1] + route[1] * leg};
    } else if (!entry && !c.inZone && distanceTo(start, goal) > 1e-6) {
        // Travel's regular stopping radius can hide a short but blocked path to safety.
        const auto [path, reached] = nav_.path(start[0], start[1], goal[0], goal[1]);
        if (reached) {
            for (int cell : path) {
                const Vec2 point = nav_.cellPos(cell);
                if (distanceTo(start, point) < 0.15) continue;
                Vec3 end{point[0], point[1], start[2]};
                if (projectMove(c, start, end)) {
                    waypoint = point;
                    fallback.move = toward(start, point);
                }
                break;
            }
        }
    }
    const Vec2 destination = entry.value_or(goal);
    std::vector<ProjectedAction> actions{{fallback, waypoint, destination}};
    if (waypoint) {
        if (fallback.boost) actions.push_back({{fallback.move, false}, waypoint, destination});
        else if (c.fire.energy > 0.1) actions.push_back({{fallback.move, true}, waypoint, destination});
    }
    if (entry) {
        const Vec2 direction = toward(start, *entry);
        actions.push_back({{direction, false}, entry, destination});
        if (c.fire.energy > 0.1) actions.push_back({{direction, true}, entry, destination});
        const WeaponInfo* laser = getWeapon(c.weapons, "laser");
        const double speed = c.walk * (laser && laser->charge > 0.0 ? c.chargeMove : 1.0);
        const double fraction = std::clamp(distanceTo(start, *entry) / (kDecision * speed), 0.0, 1.0);
        actions.push_back({{{direction[0] * fraction, direction[1] * fraction}, false}, entry, destination});
    }
    if (entry || (c.inZone && !followDetour)) {
        actions.push_back({{{}, false}, std::nullopt, destination});
    } else if (followDetour && waypoint) {
        // A brake near a turn resumes the route on the next decision.
        const double currentSpeed = std::hypot(c.fire.vel[0], c.fire.vel[1]);
        const double brakingDistance = currentSpeed * (kDecision + currentSpeed / (2.0 * c.accel));
        if (distanceTo(start, *waypoint) <= brakingDistance) {
            actions.push_back({{{}, false}, waypoint, destination});
        }
    }
    if (c.dodge[0] != 0.0 || c.dodge[1] != 0.0) {
        const Vec2 intended = waypoint ? fallback.move : toward(start, destination);
        const Vec2 mv = limitLength({0.4 * intended[0] + 0.9 * c.dodge[0], 0.4 * intended[1] + 0.9 * c.dodge[1]}, 1.0);
        actions.push_back({{mv, fallback.boost || (c.dodgeLaser && c.fire.energy > 0.1)},
                           waypoint ? waypoint : entry, destination});
    }
    return compareActions(c, actions, true);
}

Json Brain::act(const Json& obs) {
    const auto& me = obs.contains("self") && obs["self"].is_object() ? obs["self"] : Json::object();
    const auto& z = obs.contains("zone") && obs["zone"].is_object() ? obs["zone"] : Json::object();
    const auto& st = me.contains("stats") && me["stats"].is_object() ? me["stats"] : Json::object();
    now_ = numOr(obs, "t", numOr(obs, "tick", 0.0) / 20.0);

    const auto ws = parseWeapons(me);
    observe(obs, me, ws);

    const Vec3 pos = safeVec3(me, "pos");
    const Vec3 vel = safeVec3(me, "vel");
    const Vec2 curAim = safeVec2(me, "aim");
    const double x = pos[0], y = pos[1], zf = pos[2];
    const double hp = numOr(me, "hp", 1400.0);
    const double maxHp = std::max(1.0, numOr(me, "max_hp", 1400.0));
    const double hpFrac = hp / maxHp;
    const double energy = numOr(me, "energy", 100.0);
    const bool grounded = boolOr(me, "grounded", true);
    const bool inZone = boolOr(me, "in_zone", true);
    const int myCubes = intOr(me, "cubes", 0);
    const int alive = intOr(obs, "alive", 16);

    const double walkSpeed = std::max(1.0, numOr(st, "walk_speed", 11.7));
    const double turnRate = numOr(st, "turn_rate", 4.0);
    const double pitchRate = numOr(st, "pitch_rate", 3.0);
    const double minPitch = numOr(st, "min_pitch", -0.7);
    const double maxPitch = numOr(st, "max_pitch", 1.3);
    const double ownMuzzleH = numOr(st, "muzzle_height", cfgMechMuzzle_);
    const double ownHeight = numOr(st, "height", cfgMechHeight_);
    const double myArmor = numOr(st, "armor", 0.0);
    const FireState fireState{pos, vel, curAim, turnRate, pitchRate, minPitch,
                              maxPitch, ownMuzzleH, ownHeight, energy, myCubes};

    const auto movement = movementContext(me, fireState, ws);
    const auto projectiles = nearbyProjectiles(obs, movement);
    const auto incoming = forecastProjectileRisk(movement, {}, projectiles);
    const Vec2 laserDodge = laserDodgeDirection(obs, pos);

    const WeaponInfo* gun = getWeapon(ws, "gun");
    const WeaponInfo* light = getWeapon(ws, "lightning");
    const WeaponInfo* laser = getWeapon(ws, "laser");
    const WeaponInfo* mortar = getWeapon(ws, "mortar");

    const double nextRadius = numOr(z, "next_radius", 380.0);
    const bool late = nextRadius <= 70.0 || alive <= 3;

    // 1. Continuous Physics Target & Threat Evaluation
    //    CRITICAL: Decouple SHOOTING (target) from RETREAT MOVEMENT (retreat_)!
    //    Legs and torso are independent: even while running to cover, ALWAYS fire back at whoever is shooting us!
    std::vector<MechTrack*> seen;
    seen.reserve(tracks_.size());
    double totalInDPS = 0.0;
    double activeInDPS = 0.0;
    double pressure = 0.0;
    double primaryPressure = 0.0;
    Vec2 pressureBearing{};
    const double gunPressureReference = 95.0 * gunHitProb(120.0);
    for (auto& [id, t] : tracks_) {
        (void)id;
        if (now_ - t.seen > 2.5) continue;
        t.engage = false;
        t.engageScore = -1e9;
        t.currentPressure = 0.0;
        seen.push_back(&t);
        if (t.seen != now_) {
            t.los = false;
            continue;
        }
        const double wAim = attentionWeight(t, x, y);
        const double edps = enemyExpectedDPS(t, ws, myArmor, energy, t.dist, wAim > 0.7, /*burst=*/false);
        if (t.los) {
            totalInDPS += edps * wAim;
            const Vec2 towardUs = toward(t.pos, {x, y});
            const double towardSpeed = towardUs[0] * t.vel[0] + towardUs[1] * t.vel[1];
            const double commitment = fireCommitment(t, pos, towardSpeed, walkSpeed);
            const double anticipatedDistance = std::max(0.0, t.dist - std::max(0.0, towardSpeed) * 0.7);
            const double anticipatedDPS = enemyExpectedDPS(t, ws, myArmor, energy, anticipatedDistance, true, false);
            const double incomingDPS = commitment * std::max(edps, anticipatedDPS);
            if (wAim >= 0.55 || t.dist < 65.0) activeInDPS += edps * wAim;
            const double contribution = 2.0 * incomingDPS / (gunPressureReference + incomingDPS);
            t.currentPressure = contribution;
            pressure += contribution;
            primaryPressure = std::max(primaryPressure, contribution);
            pressureBearing[0] -= contribution * towardUs[0];
            pressureBearing[1] -= contribution * towardUs[1];
        } else if (t.dist < 60.0) {
            totalInDPS += edps * wAim * 0.35;
        }
    }

    MechTrack* target = nullptr;
    MechTrack* hidden = nullptr;
    double bestTargetScore = -kInf;
    double bestHiddenScore = -kInf;
    const double maxTargetDist = pracOn_ ? 200.0 : 215.0;

    for (MechTrack* t : seen) {
        if (t->seen != now_ || t->dist > maxTargetDist) continue;
        t->engageScore = engageValue(obs, me, ws, *t, totalInDPS, late);
        const double toMe = std::atan2(y - t->pos[1], x - t->pos[0]);
        const bool aimedAtMe = std::abs(angDiff(t->aim[0], toMe)) < 0.22;
        // FFA Ghost self-defense discipline: never provoke distant full-HP fights when many players remain!
        const bool selfDefense = (now_ - t->hitMe < 4.5) || (t->dist < 60.0) ||
            (aimedAtMe && t->charging && t->dist < 125.0) ||
            (t->hp < 350.0 && t->dist < 120.0) || (alive <= 2) || pracOn_;
        t->engage = (t->engageScore > 0.0) || selfDefense;
        if (!t->engage) continue;
        if (t->los) {
            if (t->engageScore > bestTargetScore) {
                bestTargetScore = t->engageScore;
                target = t;
            }
        } else if (t->engageScore > bestHiddenScore) {
            bestHiddenScore = t->engageScore;
            hidden = t;
        }
    }
    if (target) {
        stickID_ = target->id;
        stickAt_ = now_;
        const double scale = scaleOf(target->cubes);
        const FirePlan shot = planDirectFire(fireState, ws, target, centerOf(*target), target->vel,
                                             cfgMechRadius_ * scale, cfgMechHeight_ * scale, false);
        if (shot.fire[0] || shot.fire[1]) {
            const Vec2 bearing = toward(pos, {target->pos[0], target->pos[1]});
            const double off = std::abs(angDiff(target->aim[0], std::atan2(-bearing[1], -bearing[0])));
            const auto& mech = config_.contains("mech") ? config_["mech"] : Json::object();
            const double timeToTurn = off / std::max(0.1, numOr(mech, "turn_rate", 4.0));
            const double shotRate = myExpectedDPS(ws, myCubes, energy, *target, target->dist, false, false);
            const double responseRate = enemyExpectedDPS(*target, ws, myArmor, energy, target->dist, true, false);
            // A target that survives long enough to turn can add a line of fire after we shoot.
            const double returnDPS = responseRate * std::exp(-timeToTurn * shotRate / std::max(1.0, target->hp));
            const double returnPressure = 2.0 * returnDPS / (gunPressureReference + returnDPS);
            const double added = std::max(0.0, returnPressure - target->currentPressure);
            pressure += added;
            primaryPressure = std::max(primaryPressure, returnPressure);
            pressureBearing[0] += added * bearing[0];
            pressureBearing[1] += added * bearing[1];
        }
    }
    MechTrack* foe = target ? target : hidden;
    auto applyEvasion = [&](ActionChoice preferred) {
        Vec2 precaution = laserDodge;
        if (foe) {
            const Vec2 shotLane = combatRoundDodge(movement, projectiles);
            precaution[0] += shotLane[0];
            precaution[1] += shotLane[1];
        }
        if (const double length = std::hypot(precaution[0], precaution[1]); length > 0.01) {
            preferred.move = limitLength({preferred.move[0] * 0.4 + 0.9 * precaution[0] / length,
                                          preferred.move[1] * 0.4 + 0.9 * precaution[1] / length}, 1.0);
            preferred.boost = preferred.boost || (energy > 25.0 && std::hypot(laserDodge[0], laserDodge[1]) > 0.01);
        }
        return dodgeProjectiles(movement, preferred, projectiles);
    };


    const bool underFire = (activeInDPS > 45.0) || (now_ - lastHurt_ < 2.5);
    if (underFire || foe != nullptr) {
        lastThreatAt_ = now_;
    }

    // 2. Retreat hysteresis sets the default. Finishing competes with that route below.
    const bool outgunned = target && holdsKind(target->weapons, "laser") && !laser && target->dist > 55.0 && hpFrac < 0.78;
    const double retreatHP = pracOn_ ? 0.25 : 0.38;
    const double reenterHP = pracOn_ ? 0.52 : 0.78;
    const double supportingPressure = pressure - primaryPressure;
    const double spread = (pressure - std::hypot(pressureBearing[0], pressureBearing[1])) * 0.5;
    const double crossfirePressure = supportingPressure + 0.8 * std::min(spread, supportingPressure);
    const double pressureBudget = (pracOn_ ? 3.0 : 2.0) * hpFrac * hpFrac;
    const bool swamped = crossfirePressure >= pressureBudget || outgunned;
    const bool lowUnderFire = (hpFrac < retreatHP && now_ - lastThreatAt_ < 2.0);
    if (!retreat_ && !late && (lowUnderFire || swamped)) {
        retreat_ = true;
        coverAt_ = -1.0;
    }
    if (retreat_ && (late || (!swamped &&
        (hpFrac > reenterHP || (now_ - lastThreatAt_ > std::max(3.5, cfgRegenDelay_ + 1.5) && hpFrac > 0.55))))) {
        retreat_ = false;
    }

    // 3. Movement Priority Cascade (Zone / Escape / Dodge / Retreat / Fight / Unified Loot / Home)
    double mx = 0.0, my = 0.0;
    bool boost = false;
    bool jump = false;
    bool projectedMove = false;
    bool waitForCover = false;
    bool projectingCoverFire = false;
    std::string mode = "idle";
    MechTrack* farmNpc = nullptr;
    std::optional<Vec3> farmCratePos;
    std::optional<ActionChoice> checkedMovement;

    const Vec2 zg = zoneGoal(z, x, y);
    const double zdist = std::hypot(zg[0] - x, zg[1] - y);
    const double tNeed = zdist * 1.45 / walkSpeed + 4.0;
    const auto detour = grounded ? zoneDetour(z, fireState, walkSpeed,
                            std::max(1.0, numOr(st, "ground_accel", 58.5)), inZone) : std::nullopt;
    const double latestDeparture = detour ? detour->latestDeparture : zoneDeadline(z, x, y, 8.0) - tNeed;
    const bool zoneMust = !inZone || now_ + 3.0 >= latestDeparture;
    const bool zoneShould = now_ + 14.0 >= latestDeparture;
    const bool hasZoneRoute = detour || zdist > 2.0;
    const Vec2 zoneWaypoint = detour ? detour->waypoint : zg;

    if (runEscape(me, mx, my, boost, jump)) {
        roofFlying_ = false;
        mode = "escape";
    } else if (now_ < unstuckUntil_) {
        roofFlying_ = false;
        mx = unstuckDir_[0];
        my = unstuckDir_[1];
        mode = "unstuck";
    } else if (dodgeShell(obs, me, mx, my)) {
        roofFlying_ = false;
        boost = energy > 15.0;
        mode = "dodge-mortar";
    } else if (zoneMust && (!inZone || hasZoneRoute)) {
        roofFlying_ = false;
        Vec2 mv = detour ? toward(pos, zoneWaypoint) : travel(me, zg[0], zg[1]);
        boost = (!inZone || now_ >= latestDeparture) && energy > 5.0;
        if (grounded) {
            const auto context = projectionContext(obs, movement, incoming, laserDodge, target, seen, projectiles, true);
            const auto action = chooseZoneAction(context, {mv, boost}, zoneWaypoint, detour.has_value());
            mv = action.move;
            boost = action.boost;
            projectedMove = true;
        } else if (!inZone && mv == Vec2{}) {
            mv = toward(pos, zg);
        }
        mx = mv[0];
        my = mv[1];
        mode = "zone";
    } else if (retreat_ && !late) {
        roofFlying_ = false;
        const bool planFinish = target && target->dist < 120.0 && grounded &&
                                canFinishWithinProjection(fireState, ws, *target);
        std::optional<ActionChoice> baselineFight;
        std::optional<std::mt19937> fightRng;
        double fightStrafe = strafe_, fightStrafeUntil = strafeUntil_;
        if (planFinish) {
            const bool rear = showsBack(*target, x, y);
            const double burstDPS = myExpectedDPS(ws, myCubes, energy, *target, target->dist, rear, true);
            const double sustainedDPS = myExpectedDPS(ws, myCubes, energy, *target, target->dist, rear, false);
            const double fastTime = burstDPS > 15.0 ? target->hp / burstDPS : kInf;
            const double oldKillTime = fastTime <= 2.2 ? fastTime : target->hp / std::max(10.0, sustainedDPS);
            // Offer the existing fight movement first, so it breaks ties. The old shortcuts
            // only supply a candidate, and the projection still has to justify choosing it.
            if (oldKillTime < 1.35 || target->hp < 210.0 || target->hp < hp * 0.45) {
                const auto savedRng = rng_;
                const Vec2 move = fightMove(obs, me, ws, *target, seen);
                const bool charge = light && target->dist > 48.0 && target->dist < 110.0 &&
                                    energy > 55.0 && target->engageScore > 40.0;
                const bool kite = !light && holdsKind(target->weapons, "lightning") && target->dist < 62.0 &&
                                  energy > 20.0 && move != Vec2{};
                baselineFight = ActionChoice{move, charge || kite, false, true, false, true};
                fightRng = rng_;
                rng_ = savedRng;
                std::swap(fightStrafe, strafe_);
                std::swap(fightStrafeUntil, strafeUntil_);
            }
        }
        auto savedPath = baselineFight ? path_ : std::vector<int32_t>{};
        const Vec2 savedPathGoal = pathGoal_, savedCover = coverGoal_;
        const double savedPathAt = pathAt_, savedCoverAt = coverAt_;
        const std::size_t savedPathIndex = pathI_;
        const bool savedCoverOK = coverOK_;
        std::optional<Vec2> cover;
        if (coverMove(obs, me, seen, mx, my)) {
            cover = coverGoal_;
            boost = energy > 10.0 && (mx != 0.0 || my != 0.0);
            mode = "cover";
        } else if (target) {
            const Vec2 mv = fightMove(obs, me, ws, *target, seen);
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
        if (planFinish) {
            const auto context = projectionContext(obs, movement, incoming, laserDodge, target, seen, projectiles);
            const auto action = chooseFinishAction(context, {{mx, my}, boost}, cover, baselineFight);
            mx = action.move[0];
            my = action.move[1];
            boost = action.boost;
            waitForCover = action.waitForCover;
            projectedMove = true;
            projectingCoverFire = !action.stayToFight && (cover.has_value() || boost);
            if (action.stayToFight || action.finishesWhileMoving) mode = "fight";
            if (action.baselineFight) {
                retreat_ = false;
                rng_ = *fightRng;
                strafe_ = fightStrafe;
                strafeUntil_ = fightStrafeUntil;
                path_ = std::move(savedPath);
                pathGoal_ = savedPathGoal;
                pathAt_ = savedPathAt;
                pathI_ = savedPathIndex;
                coverGoal_ = savedCover;
                coverAt_ = savedCoverAt;
                coverOK_ = savedCoverOK;
            }
        }
        if (mode == "cornered") recordTrap(x, y, 2.5);
    } else if (!target && now_ - lastHurt_ < 2.5 && !late && hideFromBearing(obs, me, seen, mx, my)) {
        roofFlying_ = false;
        boost = energy > 30.0 && (mx != 0.0 || my != 0.0);
        mode = "cover";
    } else if (foe) {
        roofFlying_ = false;
        const Vec2 mv = fightMove(obs, me, ws, *foe, seen);
        mx = mv[0];
        my = mv[1];
        if (light && foe->dist > 48.0 && foe->dist < 110.0 && energy > 55.0 && foe->engageScore > 40.0) {
            boost = true;
        }
        // Anti-lightning boost-kite: stay strictly outside enemy's 45m Lightning cone!
        if (!light && holdsKind(foe->weapons, "lightning") && foe->dist < 62.0 && energy > 20.0 &&
            (mx != 0.0 || my != 0.0)) {
            boost = true;
        }
        mode = "fight";
        // Safe opportunistic mid-fight cube grab (only when valuable & not facing charging laser / multi-foe!)
        bool enemyCharging = false;
        for (const MechTrack* e : seen) {
            if (e->seen == now_ && e->los && e->charging) {
                enemyCharging = true;
                break;
            }
        }
        if (crossfirePressure < 0.5 * hpFrac && !enemyCharging && cubeValue(myCubes, hpFrac) >= 50.0) {
            const ZoneCircle curZ = curZone(z);
            double bestCubeD = pracOn_ ? (hpFrac > 0.50 ? 68.0 : 38.0) : (hpFrac > 0.50 ? 40.0 : 20.0);
            std::optional<Vec3> bc;
            for (const auto& [cid, c] : cubes_) {
                (void)cid;
                const double d = std::hypot(c.pos[0] - x, c.pos[1] - y);
                if (d < bestCubeD && std::abs(c.pos[2] - zf) < 4.0 &&
                    std::hypot(c.pos[0] - curZ.x, c.pos[1] - curZ.y) < curZ.r - 4.0) {
                    const double dFoe = std::hypot(c.pos[0] - foe->pos[0], c.pos[1] - foe->pos[1]);
                    if (d < 14.0 || d < dFoe * 0.95) {
                        bestCubeD = d;
                        bc = c.pos;
                    }
                }
            }
            if (bc) {
                if (projectiles.empty()) {
                    const Vec2 mv = travel(me, (*bc)[0], (*bc)[1]);
                    mx = mv[0];
                    my = mv[1];
                    mode = "fight:cube";
                } else {
                    auto originalPath = path_;
                    const Vec2 originalGoal = pathGoal_;
                    const double originalTime = pathAt_;
                    const std::size_t originalIndex = pathI_;
                    const Vec2 mv = travel(me, (*bc)[0], (*bc)[1]);
                    const auto pickup = applyEvasion({mv, boost});
                    const auto fight = applyEvasion({{mx, my}, boost});
                    const double pickupDamage = forecastProjectileRisk(movement, pickup, projectiles).damage;
                    const double fightDamage = forecastProjectileRisk(movement, fight, projectiles).damage;
                    if (pickupDamage <= fightDamage) {
                        checkedMovement = pickup;
                        mode = "fight:cube";
                    } else {
                        checkedMovement = fight;
                        path_ = std::move(originalPath);
                        pathGoal_ = originalGoal;
                        pathAt_ = originalTime;
                        pathI_ = originalIndex;
                    }
                }
            }
        }
    } else if (zoneShould && hasZoneRoute) {
        roofFlying_ = false;
        const Vec2 mv = detour ? toward(pos, zoneWaypoint) : travel(me, zg[0], zg[1]);
        mx = mv[0];
        my = mv[1];
        mode = "zone";
    } else {
        if (const auto lc = chooseLoot(obs, me, ws, hpFrac, inZone, incoming.damage > 0.0)) {
            mode = "loot:" + lc->kind;
            if (lc->isRoof && zf < lc->pos[2] - 1.5) {
                // Rooftop jet-climb controller
                const double dLaunch = std::hypot(lc->roofLaunch[0] - x, lc->roofLaunch[1] - y);
                if (grounded) {
                    roofFlying_ = false;
                    if (dLaunch > 2.5) {
                        const Vec2 mv = travel(me, lc->roofLaunch[0], lc->roofLaunch[1]);
                        mx = mv[0];
                        my = mv[1];
                        mode = "roof:approach";
                    } else if (energy < lc->climbEnergy + 12.0) {
                        mx = 0.0;
                        my = 0.0;
                        mode = "roof:charge";
                    } else {
                        jump = true;
                        roofFlying_ = true;
                        mode = "roof:launch";
                    }
                } else {
                    roofFlying_ = true;
                    const double dx = lc->pos[0] - x, dy = lc->pos[1] - y;
                    const double dl = std::hypot(dx, dy);
                    if (zf < lc->pos[2] + 1.2) {
                        boost = energy > 1.0;
                        // Strictly clear the roof rim (+1.2m) vertically BEFORE gliding horizontally onto roof!
                        mx = 0.0;
                        my = 0.0;
                    } else if (dl > 0.5) {
                        mx = dx / dl;
                        my = dy / dl;
                        // Feather jetpack while gliding across the gap so gravity doesn't drop us below roof rim
                        boost = (zf < lc->pos[2] + 2.5 && arena_.topSurface(x, y) < lc->pos[2] - 1.0) && energy > 1.0;
                    }
                    mode = "roof:jet";
                }
            } else {
                roofFlying_ = false;
                if (lc->kind == "npc") {
                    auto nit = npcs_.find(lc->id);
                    if (nit != npcs_.end() && now_ - nit->second.seen < 1.0 && nit->second.los && nit->second.dist < 75.0) {
                        farmNpc = &nit->second;
                        const Vec2 mv = fightMove(obs, me, ws, nit->second, seen);
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
                    if (lc->kind == "drop" && std::hypot(lc->pos[0] - x, lc->pos[1] - y) < 3.5) {
                        mx = 0.0;
                        my = 0.0;
                        mode = "cap-drop";
                    }
                }
            }
        } else if (zdist > 3.0 && nextRadius <= 30.0) {
            roofFlying_ = false;
            const Vec2 mv = travel(me, zg[0], zg[1]);
            mx = mv[0];
            my = mv[1];
            mode = "zone-walk";
        } else {
            roofFlying_ = false;
            if (const auto hm = findHome(obs, me); hm && std::hypot((*hm)[0] - x, (*hm)[1] - y) > 3.0) {
                const Vec2 mv = travel(me, (*hm)[0], (*hm)[1]);
                mx = mv[0];
                my = mv[1];
                // Silent walk (< 5.0 m/s step_speed) when safe so we emit zero footsteps!
                if (!underFire && now_ - lastHurt_ > 6.0) {
                    const double q = std::min(1.0, 4.6 / walkSpeed);
                    mx *= q;
                    my *= q;
                }
                mode = "home";
            }
        }
    }

    // Urgent/projected actions already own movement. Ordinary movement is checked before committing it.
    if (!projectedMove && mode != "unstuck" && mode != "escape" && mode != "dodge-mortar" &&
        mode != "roof:jet" && mode != "roof:launch") {
        const ActionChoice action = checkedMovement ? *checkedMovement : applyEvasion({{mx, my}, boost});
        mx = action.move[0];
        my = action.move[1];
        boost = action.boost;
    }

    // 5. Fast Physical Unstuck Detector
    if (mode == "escape" || mode == "roof:charge" || mode == "roof:jet" || mode == "cap-drop") {
        stuckSince_ = -1.0;
    } else if ((projectedMove ? std::hypot(mx, my) * walkSpeed > 1.2 : (mx != 0.0 || my != 0.0)) &&
               std::hypot(vel[0], vel[1]) < 1.2 && grounded) {
        if (stuckSince_ < 0.0) {
            stuckSince_ = now_;
        } else if (now_ - stuckSince_ > 0.7) {
            std::uniform_real_distribution<double> u01(0.0, 1.0);
            const double a = u01(rng_) * kTau;
            unstuckDir_ = {std::cos(a), std::sin(a)};
            unstuckUntil_ = now_ + 0.5;
            stuckSince_ = -1.0;
            path_.clear();
            roofFlying_ = false;
            recordTrap(x, y, 2.0);
            jump = true;
        }
    } else {
        stuckSince_ = -1.0;
    }

    // 6. Combat Aiming, Slew Prediction, Lethal Laser, Indirect Mortar & Regen Silence Gate
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

    // Strict regen silence gate for non-lethal / non-self-defense PvE shots
    if (!isMech && !defendNpc && (retreat_ || regenBreakCost(hpFrac, inZone) > 30.0)) {
        shootable = false;
    }

    std::array<bool, 2> fire{false, false};
    if (shootable) {
        const FirePlan shot = planDirectFire(fireState, ws, target, tp, tv, tr, th,
                                             (mode == "cover" || projectingCoverFire) && (mx != 0.0 || my != 0.0));
        aim = shot.aim;
        fire = shot.fire;
        haveAim = true;
        if (waitForCover) fire.fill(false);
    }

    // Indirect mortar blind-fire over buildings using 5s memory_s real-time enemy coordinates!
    if (!target && mortar && hidden && now_ - hidden->seen < 3.0 &&
        mortar->cooldown <= 0.25 && hidden->dist >= 20.0 && hidden->dist <= 195.0 &&
        regenBreakCost(hpFrac, inZone) < 35.0) {
        const Vec3 tube{muzzle[0], muzzle[1], zf + ownHeight + 0.5};
        if (const auto ma = mortarAim(tube, hidden->pos, hidden->vel, mortar->speed, mortar->gravity,
                                      minPitch, maxPitch, /*allowHighArc=*/true)) {
            aim = *ma;
            haveAim = true;
            const double ey = std::max(0.0, std::abs(angDiff((*ma)[0], curAim[0])) - turnRate * 0.05);
            const double ep = std::max(0.0, std::abs((*ma)[1] - curAim[1]) - pitchRate * 0.05);
            if (ey < 0.05 && ep < 0.04) {
                const int mSlot = findWeaponSlot(ws, "mortar");
                if (mSlot >= 0) fire[static_cast<std::size_t>(mSlot)] = true;
            }
        }
    }

    // Behind-wall laser pre-charge ONLY when actively peeking an engaged close foe (never while retreating/escaping!)
    if (!target && foe && laser && mode == "fight" && !retreat_) {
        const int lSlot = findWeaponSlot(ws, "laser");
        if (lSlot >= 0 && laser->cooldown <= 0.05 && foe->dist < std::min(130.0, laser->range) &&
            laser->charge < laser->fullCharge + laser->holdS - 0.35 &&
            regenBreakCost(hpFrac, inZone) < 40.0) {
            fire[static_cast<std::size_t>(lSlot)] = true;
        }
    }

    // 7. Multi-Threat 180-deg Optimal Front-Armor Yaw OR Active Vision Scanning
    if (!haveAim) {
        if (const auto armorYaw = bestArmorYaw(me, incoming, ws, seen)) {
            aim[0] = *armorYaw;
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

    // 8. Clean Early Reload (never reloads mid-burst during inter-shot cooldown!)
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
        {"boost", boost && (mx != 0.0 || my != 0.0 || roofFlying_ || escFlying_)},
        {"jump", jump},
        {"fire", {fire[0], fire[1]}},
        {"reload", reload},
    };

    // 9. Explicit Weapon Pickup / Upgrade Swap (function-driven!)
    if (const auto pit = obs.find("pickups"); pit != obs.end() && pit->is_array()) {
        for (const auto& p : *pit) {
            if (!p.is_object() || !p.contains("id") || !p["id"].is_number()) continue;
            const Vec3 ppos = safeVec3(p, "pos");
            if (std::hypot(ppos[0] - x, ppos[1] - y) > cfgLootPickupRadius_ - 0.3 ||
                std::abs(ppos[2] - zf) > 3.6) {
                continue;
            }
            const std::string kind = strOr(p, "kind");
            if (!wantPickup(ws, kind, nextRadius)) continue;
            if (ws[1] && ws[0]) {
                if (ws[1]->kind == "laser" && ws[1]->charge > 0.0) break;
                action["drop"] = 1;
            }
            action["take"] = static_cast<int>(p["id"].get<double>());
            break;
        }
    }

    // 10. Dynamic Context-Aware Upgrade Selection
    if (const auto oit = obs.find("offer"); oit != obs.end() && !oit->is_null()) {
        action["pick"] = pickUpgrade(*oit, ws, nextRadius);
    }

    if (cfgDebugMaxChars_ >= 16) {
        action["debug"] = {{"label", mode}};
    }
    return action;
}

}  // namespace mr
