#pragma once

#include "arena.hpp"
#include "common.hpp"
#include "navgrid.hpp"

#include <array>
#include <cstdint>
#include <limits>
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
    bool weaponsKnown = false;
    bool charging = false;
    double seen = -100.0;
    double hitMe = -100.0;
    double dist = 0.0;
    bool los = false;
    double closing = 0.0;
    double currentPressure = 0.0;
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

struct WarEvent {
    Vec2 pos{};
    double weight = 1.0;
    double at = -100.0;
    bool isDeath = false;
};

struct RosterEntry {
    bool hasGun = false;
    bool hasLaser = false;
    bool hasLightning = false;
    bool hasMortar = false;
    int cubes = 0;
    Vec3 lastPos{};
    double lastSeen = -100.0;
    double loadoutSeen = -std::numeric_limits<double>::infinity();
    bool dead = false;
};

struct TrapZone {
    Vec2 pos{};
    double at = -100.0;
    double weight = 2.0;
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

    using Weapons = std::array<std::optional<WeaponInfo>, 2>;

    struct FireState {
        Vec3 pos{};
        Vec3 vel{};
        Vec2 aim{};
        double turnRate = 4.0;
        double pitchRate = 3.0;
        double minPitch = -0.7;
        double maxPitch = 1.3;
        double muzzleHeight = 7.0;
        double height = 9.0;
        double energy = 100.0;
        int cubes = 0;
    };

    struct FirePlan {
        Vec2 aim{};
        Vec2 gunAim{};
        Vec2 hitscanAim{};
        std::array<bool, 2> fire{};
        double yawTolerance = 0.0;
        double pitchTolerance = 0.0;
    };

    struct ActionChoice {
        Vec2 move{};
        bool boost = false;
        bool waitForCover = false;
        bool stayToFight = false;
        bool finishesWhileMoving = false;
        bool baselineFight = false;
    };

    struct ProjectionContext;
    struct ProjectedAction;
    struct Projection;
    struct Projectile {
        Vec3 pos;
        Vec3 vel;
        double radius;
        double damage;
    };

    struct ProjectileRisk {
        double damage = 0.0;
        double firstHit = std::numeric_limits<double>::infinity();
        Vec2 side{};
        double bearing = 0.0;
    };

    FirePlan planDirectFire(const FireState& state, const Weapons& weapons, const MechTrack* target,
                            const Vec3& point, const Vec3& velocity, double radius, double height,
                            bool movingToCover) const;
    ProjectionContext movementContext(const Json& me, const FireState& fire, const Weapons& weapons) const;
    ProjectionContext projectionContext(const Json& obs, const ProjectionContext& movement,
                                        const ProjectileRisk& incoming, Vec2 laserDodge,
                                        const MechTrack* target,
                                        const std::vector<MechTrack*>& seen,
                                        const std::vector<Projectile>& projectiles, bool enteringZone = false) const;
    std::vector<Projectile> nearbyProjectiles(const Json& obs, const ProjectionContext& movement) const;
    std::optional<double> projectileContact(const Projectile& projectile, double stepStart, const Vec3& before,
                                            const Vec3& after, double radius, double height) const;
    ProjectileRisk forecastProjectileRisk(const ProjectionContext& movement, ActionChoice action,
                                          const std::vector<Projectile>& projectiles) const;
    ActionChoice dodgeProjectiles(const ProjectionContext& movement, ActionChoice fallback,
                                  const std::vector<Projectile>& projectiles) const;
    Vec2 combatRoundDodge(const ProjectionContext& movement, const std::vector<Projectile>& projectiles) const;
    bool advanceGroundMotion(const ProjectionContext& context, FireState& state, Vec2 move, double speed) const;
    void projectAction(const ProjectionContext& context, const ProjectedAction& action,
                       Projection& state, bool pursuit, int decision) const;
    ActionChoice compareActions(const ProjectionContext& context, const std::vector<ProjectedAction>& actions,
                                bool enteringZone) const;
    ActionChoice chooseFinishAction(const ProjectionContext& context, ActionChoice fallback,
                                    std::optional<Vec2> cover, std::optional<ActionChoice> baselineFight);
    ActionChoice chooseZoneAction(const ProjectionContext& context, ActionChoice fallback, Vec2 goal,
                                  bool followDetour);
    bool projectMove(const ProjectionContext& context, const Vec3& from, Vec3& to) const;
    bool canFinishWithinProjection(const FireState& state, const Weapons& weapons, const MechTrack& target) const;

    double scaleOf(int cubes) const;
    Vec3 centerOf(const MechTrack& t) const;
    Vec3 eyeOf(const MechTrack& t) const;
    bool showsBack(const MechTrack& t, double x, double y) const;

    // Continuous value, risk & map-memory functions (zero magic thresholds)
    double soundHeat(double px, double py) const;
    double warzoneHeat(double px, double py) const;
    double trapPenalty(double px, double py) const;
    void markVisited(double px, double py);
    double unvisitedScore(double px, double py) const;
    void recordTrap(double px, double py, double weight);
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
    double fireCommitment(const MechTrack& t, const Vec3& pos, double towardSpeed, double walkSpeed) const;
    double engageValue(const Json& obs, const Json& me,
                       const std::array<std::optional<WeaponInfo>, 2>& ws,
                       const MechTrack& t, double inDPS, bool late) const;
    std::optional<double> bestArmorYaw(const Json& me, const ProjectileRisk& incoming,
                                       const std::array<std::optional<WeaponInfo>, 2>& ws,
                                       const std::vector<MechTrack*>& seen) const;

    void observe(const Json& obs, const Json& me, const std::array<std::optional<WeaponInfo>, 2>& ws);

    ZoneCircle zoneAt(const Json& z, double t) const;
    ZoneCircle curZone(const Json& z) const;
    double zoneDeadline(const Json& z, double x, double y, double pad) const;
    double zonePenalty(const Json& z, double x, double y) const;
    Vec2 zoneGoal(const Json& z, double x, double y) const;

    struct ZoneDetour {
        Vec2 waypoint;
        double latestDeparture;
    };
    std::optional<ZoneDetour> zoneDetour(const Json& z, const FireState& state,
                                          double speed, double accel, bool inZone);

    ZoneCircle detourCircle_{};
    double detourPhaseEnd_ = -1.0;
    double detourRetryAt_ = -1.0;
    std::vector<Vec2> detourPath_;
    std::size_t detourIndex_ = 0;

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
                                         double hpFrac, bool inZone, bool incomingRound);
    double homeCost(const Json& obs, const Json& me, double px, double py) const;
    std::optional<Vec2> findHome(const Json& obs, const Json& me);
    Vec2 laserDodgeDirection(const Json& obs, Vec3 pos) const;

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

    // Persistent memory maps (4-layer dynamic map memory)
    std::unordered_map<int, MechTrack> tracks_;
    std::unordered_map<int, MechTrack> npcs_;
    std::unordered_map<int, KnownItem> crates_;
    std::unordered_map<int, KnownItem> cubes_;
    std::unordered_map<int, KnownItem> pickups_;
    std::unordered_map<int, double> badLoot_;
    std::vector<ThreatPing> pings_;
    std::vector<WarEvent> warEvents_;
    std::unordered_map<int, RosterEntry> roster_;
    std::vector<TrapZone> trapZones_;
    double lastTrapAt_ = -100.0;
    double memCellSize_ = 20.0;
    int memNX_ = 1;
    int memNY_ = 1;
    std::vector<float> visitedAt_;

    double now_ = 0.0;
    double laserPresence_ = 0.65;  // Realistic OSU lobby prior (lasers spawn at t=0)

    // Practice vs FFA auto-detection
    bool initDone_ = false;
    bool pracOn_ = false;
    bool notPractice_ = false;
    int startAlive_ = 16;
    int kills_ = 0;
    int otherDeadTicks_ = 0;

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
    double laserShortUntil_ = -100.0;
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
