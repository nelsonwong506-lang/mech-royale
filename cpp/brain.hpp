#pragma once

#include "arena.hpp"
#include "common.hpp"
#include "navgrid.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

namespace mr {

struct WeaponInfo {
    std::string kind;
    double cooldown = 0.0;
    double ammo = 0.0;
    double reload = 0.0;
    double charge = 0.0;
    // Dynamic stats
    double damage = 45.0;
    double fireInterval = 0.24;
    double speed = 252.0;
    double range = 230.0;
    double mag = 12.0;
    double reloadS = 2.4;
    double splashRadius = 7.0;
    double gravity = 22.0;
    double fullCharge = 0.75;
    double minCharge = 0.15;
    double minFrac = 0.35;
    double holdS = 2.0;
    double cookoff = 0.6;
    double coneDeg = 30.0;
    double energyCost = 3.0;
};

struct MechTrack {
    int id = -1;
    std::string kind;
    Vec3 pos{};
    Vec3 vel{};
    Vec2 aim{};
    double hp = 1400.0;
    double maxHp = 1400.0;
    int cubes = 0;
    std::vector<std::string> weapons;
    bool charging = false;
    double seen = -100.0;
    double hitMe = -100.0;
    double dist = 0.0;
    bool los = false;
    double closing = 0.0;
    bool engage = false;
    double engageScore = -1e9;
};

struct KnownItem {
    int id = -1;
    std::string kind;
    Vec3 pos{};
    double hp = 0.0;
    double seen = -100.0;
};

struct ThreatPing {
    Vec2 pos{};
    double weight = 1.0;
    double at = -100.0;
    double radius = 75.0;
    bool isLaser = false;
    bool exact = false;
};

struct ZoneCircle {
    double x = 0.0;
    double y = 0.0;
    double r = 560.0;
};

class Brain {
public:
    explicit Brain(const Json& hello);
    Json act(const Json& obs);

private:
    struct LootChoice {
        std::string kind;
        int id = -1;
        Vec3 pos{};
        double utility = 0.0;
        bool isRoof = false;
        Vec2 roofLaunch{};
        double climbEnergy = 0.0;
    };

    double scaleOf(int cubes) const;
    Vec3 centerOf(const MechTrack& t) const;
    Vec3 eyeOf(const MechTrack& t) const;
    bool showsBack(const MechTrack& t, double x, double y) const;

    // Continuous value & risk functions (zero magic thresholds)
    double soundHeat(double px, double py) const;
    double regenBreakCost(double hpFrac, bool inZone) const;
    double cubeValue(int cubes, double hpFrac) const;
    double weaponPower(std::string_view kind, double nextRadius) const;
    double weaponGain(const std::array<std::optional<WeaponInfo>, 2>& ws,
                      std::string_view kind, double nextRadius) const;
    bool wantPickup(const std::array<std::optional<WeaponInfo>, 2>& ws,
                    const std::string& kind, double nextRadius) const;
    int pickUpgrade(const Json& offer, const std::array<std::optional<WeaponInfo>, 2>& ws,
                    double nextRadius) const;

    // Continuous physics combat functions
    double myExpectedDPS(const std::array<std::optional<WeaponInfo>, 2>& ws,
                         int myCubes, double energy, const MechTrack& target,
                         double dist, bool rearShot, bool burst) const;
    double enemyExpectedDPS(const MechTrack& enemy,
                            const std::array<std::optional<WeaponInfo>, 2>& myWs,
                            double myArmor, double myEnergy, double dist,
                            bool aimingAtMe, bool burst) const;
    double attentionWeight(const MechTrack& t, double x, double y) const;
    double engageValue(const Json& obs, const Json& me,
                       const std::array<std::optional<WeaponInfo>, 2>& ws,
                       const MechTrack& t, double inDPS, bool late) const;
    std::optional<double> bestArmorYaw(const Json& obs, const Json& me,
                                       const std::array<std::optional<WeaponInfo>, 2>& ws,
                                       const std::vector<MechTrack*>& seen) const;

    void observe(const Json& obs, const Json& me, const std::array<std::optional<WeaponInfo>, 2>& ws);

    ZoneCircle zoneAt(const Json& z, double t) const;
    ZoneCircle curZone(const Json& z) const;
    double zoneDeadline(const Json& z, double x, double y, double pad) const;
    double zonePenalty(const Json& z, double x, double y) const;
    Vec2 zoneGoal(const Json& z, double x, double y) const;

    bool runEscape(const Json& me, double& mx, double& my, bool& boost, bool& jump);
    bool dodgeShell(const Json& obs, const Json& me, double& mx, double& my) const;
    Vec2 travel(const Json& me, double gx, double gy);
    std::optional<Vec2> findRoofLaunch(double ix, double iy, double iz, double ownHeight) const;

    Vec2 fightMove(const Json& obs, const Json& me, const std::array<std::optional<WeaponInfo>, 2>& ws,
                   const MechTrack& tg, const std::vector<MechTrack*>& seen);
    bool coverMove(const Json& obs, const Json& me, const std::vector<MechTrack*>& seen, double& mx, double& my);
    bool hideFromBearing(const Json& obs, const Json& me, const std::vector<MechTrack*>& seen, double& mx, double& my);
    std::optional<LootChoice> chooseLoot(const Json& obs, const Json& me,
                                         const std::array<std::optional<WeaponInfo>, 2>& ws,
                                         double hpFrac, bool inZone);
    double homeCost(const Json& obs, const Json& me, double px, double py) const;
    std::optional<Vec2> findHome(const Json& obs, const Json& me);
    void dodgeBullets(const Json& obs, const Json& me, double& dx, double& dy, bool& enemyLaser) const;

    Vec2 leadAim(const Vec3& muzzle, const Vec3& tp, const Vec3& tv, double speed, bool grounded) const;
    std::optional<Vec2> mortarAim(const Vec3& tube, const Vec3& foot, const Vec3& tv,
                                  double v, double g, double minPitch, double maxPitch,
                                  bool allowHighArc = false) const;

    Json config_;
    Arena arena_;
    NavGrid nav_;
    std::mt19937 rng_;

    // Config cached physics constants
    double cfgMechRadius_ = 2.5;
    double cfgMechHeight_ = 9.0;
    double cfgMechEye_ = 8.0;
    double cfgMechMuzzle_ = 7.0;
    double cfgFrontArcDeg_ = 180.0;
    double cfgRearMult_ = 1.5;
    double cfgRegenDelay_ = 5.0;
    double cfgRegenPerS_ = 45.0;
    double cfgEnergyRegen_ = 18.0;
    double cfgMaxRiseSpeed_ = 15.0;
    double cfgNpcRadius_ = 1.8;
    double cfgNpcHeight_ = 3.5;
    double cfgCrateRadius_ = 1.4;
    double cfgCrateHeight_ = 2.4;
    double cfgSizePerCube_ = 0.05;
    double cfgDmgPerCube_ = 0.04;
    int cfgCubeStatCap_ = 10;
    std::vector<int> cfgUpgradeThresholds_{1, 3, 6, 10, 15, 21, 28, 36};
    double cfgLootPickupRadius_ = 4.0;
    double cfgMortarGravity_ = 22.0;
    double cfgMortarSplash_ = 7.0;
    int cfgDebugMaxChars_ = 0;

    // Persistent memory maps
    std::unordered_map<int, MechTrack> tracks_;
    std::unordered_map<int, MechTrack> npcs_;
    std::unordered_map<int, KnownItem> crates_;
    std::unordered_map<int, KnownItem> cubes_;
    std::unordered_map<int, KnownItem> pickups_;
    std::unordered_map<int, double> badLoot_;
    std::vector<ThreatPing> pings_;

    double now_ = 0.0;
    double laserPresence_ = 0.25;  // Continuous estimate of laser prevalence in lobby [0, 1]

    // Path state
    std::vector<int32_t> path_;
    Vec2 pathGoal_{};
    double pathAt_ = -100.0;
    std::size_t pathI_ = 0;

    // Strafe / combat / retreat state
    double strafe_ = 1.0;
    double strafeUntil_ = -100.0;
    bool retreat_ = false;
    double coverAt_ = -100.0;
    Vec2 coverGoal_{};
    bool coverOK_ = false;
    int stickID_ = -1;
    double stickAt_ = -100.0;

    // Damage / combat regen / threat / loot tracking
    double lastHurt_ = -100.0;
    double hurtBearing_ = 0.0;
    double lastCombatAt_ = -100.0;  // max(lastHurt, lastHit) -> exact 5s regen clock!
    double npcHurt_ = -100.0;
    double npcBearing_ = 0.0;
    double lastThreatAt_ = -100.0;
    double lastLaserSoundAt_ = -100.0;
    double laserSoundBearing_ = 0.0;
    int lootID_ = -1;
    double lootSince_ = -100.0;
    bool roofFlying_ = false;

    // Zone hold tracking
    ZoneCircle hold_{};
    double holdStart_ = -1.0;

    // Home / unstuck / escape state
    Vec2 homeXY_{};
    double homeAt_ = -100.0;
    bool homeSet_ = false;
    double stuckSince_ = -1.0;
    double unstuckUntil_ = -100.0;
    Vec2 unstuckDir_{};
    Vec2 lastAim_{0.0, 0.0};

    std::optional<EscapeRoute> escRoute_;
    double escAt_ = -100.0;
    bool escFlying_ = false;
};

}  // namespace mr
