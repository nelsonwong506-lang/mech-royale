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
    double speed = 252.0;
    double range = 230.0;
    double mag = 12.0;
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
};

struct KnownItem {
    int id = -1;
    std::string kind;
    Vec3 pos{};
    double hp = 0.0;
    double seen = -100.0;
};

struct ZoneCircle {
    double x = 0.0;
    double y = 0.0;
    double r = 560.0;
};

struct BrainParams {
    double retreatHP = 0.38;
    double recoverHP = 0.80;
    double gunRange = 85.0;
    double lightRange = 28.0;
    double laserRange = 85.0;
    double engageDist = 60.0;
    bool farmNPC = true;
    bool explore = false;
    bool ghost = true;
    double homeExpo = 120.0;
    double lootRadius = 90.0;
    double crateUntil = 150.0;
    bool dodge = true;
    bool noMortar = true;
    bool peek = true;
    bool laserFirst = true;
    bool crowdHide = true;
    double laserSeek = 0.0;
    bool lethal = false;
    double lethalK = 0.72;
    double stick = 0.0;
    double hpW = 60.0 / 1400.0;
    double lowBonus = 0.0;
    double maxTarget = 215.0;
    bool loseRule = false;
    double fightLoot = 0.0;
    double farmHP = 0.70;
    bool laserUp = false;
    double finish = 0.0;
    double finishHP = 0.38;
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
        double value = 0.0;
    };

    static BrainParams defaultParams();
    static BrainParams squadParams(BrainParams base);

    double scaleOf(int cubes) const;
    Vec3 centerOf(const MechTrack& t) const;
    Vec3 eyeOf(const MechTrack& t) const;
    bool showsBack(const MechTrack& t, double x, double y) const;
    bool wantPickup(const std::array<std::optional<WeaponInfo>, 2>& ws, const std::string& kind) const;
    int pickUpgrade(const Json& offer) const;

    void observe(const Json& obs, const Json& me, const std::array<std::optional<WeaponInfo>, 2>& ws);

    ZoneCircle zoneAt(const Json& z, double t) const;
    ZoneCircle curZone(const Json& z) const;
    double zoneDeadline(const Json& z, double x, double y, double pad) const;
    double zonePenalty(const Json& z, double x, double y) const;
    Vec2 zoneGoal(const Json& z, double x, double y) const;

    bool runEscape(const Json& me, double& mx, double& my, bool& boost, bool& jump);
    bool dodgeShell(const Json& obs, const Json& me, double& mx, double& my) const;
    Vec2 travel(const Json& me, double gx, double gy);
    double prefRange(const Json& me, const std::array<std::optional<WeaponInfo>, 2>& ws, const MechTrack& t) const;
    Vec2 fightMove(const Json& obs, const Json& me, const std::array<std::optional<WeaponInfo>, 2>& ws,
                   const MechTrack& tg, const std::vector<MechTrack*>& seen, double wantLos);
    bool coverMove(const Json& obs, const Json& me, const std::vector<MechTrack*>& seen, double& mx, double& my);
    bool hideFromBearing(const Json& obs, const Json& me, const std::vector<MechTrack*>& seen, double& mx, double& my);
    std::optional<LootChoice> chooseLoot(const Json& obs, const Json& me,
                                         const std::array<std::optional<WeaponInfo>, 2>& ws,
                                         double hpFrac, bool danger);
    Vec2 exploreMove(const Json& obs, const Json& me);
    double homeCost(const Json& obs, const Json& me, double px, double py) const;
    std::optional<Vec2> findHome(const Json& obs, const Json& me);
    void dodgeBullets(const Json& obs, const Json& me, double& dx, double& dy, bool& enemyLaser) const;

    Vec2 leadAim(const Vec3& muzzle, const Vec3& tp, const Vec3& tv, double speed, bool grounded) const;
    std::optional<Vec2> mortarAim(const Vec3& tube, const Vec3& foot, const Vec3& tv,
                                  double v, double g, double minPitch, double maxPitch) const;

    Json config_;
    Arena arena_;
    NavGrid nav_;
    std::mt19937 rng_;
    BrainParams baseP_;
    BrainParams p_;

    // Config cached constants
    double cfgMechRadius_ = 2.5;
    double cfgMechHeight_ = 9.0;
    double cfgMechEye_ = 8.0;
    double cfgMechMuzzle_ = 7.0;
    double cfgFrontArcDeg_ = 180.0;
    double cfgRearMult_ = 1.5;
    double cfgNpcRadius_ = 1.8;
    double cfgNpcHeight_ = 3.5;
    double cfgCrateRadius_ = 1.4;
    double cfgCrateHeight_ = 2.4;
    double cfgSizePerCube_ = 0.05;
    double cfgDmgPerCube_ = 0.04;
    int cfgCubeStatCap_ = 10;
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

    double now_ = 0.0;

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
    double loseAt_ = -100.0;
    int stickID_ = -1;
    double stickAt_ = -100.0;
    double shortUntil_ = -100.0;

    // Damage / threat / loot tracking
    double lastHurt_ = -100.0;
    double hurtBearing_ = 0.0;
    double npcHurt_ = -100.0;
    double npcBearing_ = 0.0;
    double lastThreatAt_ = -100.0;
    int lootID_ = -1;
    double lootSince_ = -100.0;

    // Zone hold tracking
    ZoneCircle hold_{};
    double holdStart_ = -1.0;

    // Home / explore / unstuck / escape state
    Vec2 homeXY_{};
    double homeAt_ = -100.0;
    bool homeSet_ = false;
    Vec2 exploreXY_{};
    double exploreUntil_ = -100.0;
    double stuckSince_ = -1.0;
    double unstuckUntil_ = -100.0;
    Vec2 unstuckDir_{};
    Vec2 lastAim_{0.0, 0.0};

    std::optional<EscapeRoute> escRoute_;
    double escAt_ = -100.0;
    bool escFlying_ = false;

    // Practice vs FFA auto-detection
    bool initDone_ = false;
    bool pracOn_ = false;
    bool confirmedPractice_ = false;
    bool openOn_ = false;
    bool notPractice_ = false;
    int startAlive_ = 16;
    int kills_ = 0;
    int otherDeadTicks_ = 0;
};

}  // namespace mr
