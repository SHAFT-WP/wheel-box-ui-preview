import { casToTas } from "../../../common/airspeed/airspeed-v0.1.mjs";
import { coordinatedTurnFromTasAndLoadFactor } from "../../../common/maneuvers/turn-performance/turn-performance-v0.1.mjs";

export const WHEEL_BOX_GEOMETRY_MODEL_V0_4 = Object.freeze({
  id: "wheel-box-bdp-min-wheel-bilateral-closure-v0.4",
  version: "0.4.0",
  status: "Work / Pure Calculation / Not Official",
});

const BOX_ANGLE_OFF_DEG = 90;
const BASE_TURN_HEADING_CHANGE_DEG = 90;
const DEFAULT_BASE_TURN_LOAD_FACTOR_G = 2;
const EPSILON_NM = 1e-12;
const BDP_ALIGNMENT_TOLERANCE_NM = 1e-9;
const HEADING_ALIGNMENT_TOLERANCE_DEG = 1e-10;
const SUPPORTED_DIRECTIONS = Object.freeze(["RIGHT", "LEFT"]);

function requireFinite(name, value) {
  if (typeof value !== "number" || !Number.isFinite(value)) {
    throw new TypeError(`${name} must be a finite number`);
  }
  return value;
}

function requirePositive(name, value) {
  requireFinite(name, value);
  if (!(value > 0)) throw new RangeError(`${name} must be > 0`);
  return value;
}

function requireDirection(direction) {
  if (!SUPPORTED_DIRECTIONS.includes(direction)) {
    throw new RangeError("direction must be RIGHT or LEFT");
  }
  return direction;
}

function normalizeBearingDeg(value) {
  requireFinite("bearingDeg", value);
  return ((value % 360) + 360) % 360;
}

function headingResidualDeg(fromDeg, toDeg) {
  return ((toDeg - fromDeg + 540) % 360) - 180;
}

function pointFromBearingRange(bearingDeg, rangeNm) {
  const rad = (bearingDeg * Math.PI) / 180;
  return {
    eastNm: rangeNm * Math.sin(rad),
    northNm: rangeNm * Math.cos(rad),
  };
}

function translatePoint(point, bearingDeg, distanceNm) {
  const delta = pointFromBearingRange(bearingDeg, distanceNm);
  return {
    eastNm: point.eastNm + delta.eastNm,
    northNm: point.northNm + delta.northNm,
  };
}

function vectorBetween(from, to) {
  return {
    eastNm: to.eastNm - from.eastNm,
    northNm: to.northNm - from.northNm,
  };
}

function vectorRangeNm(vector) {
  return Math.hypot(vector.eastNm, vector.northNm);
}

function dot(a, b) {
  return a.eastNm * b.eastNm + a.northNm * b.northNm;
}

function cross(a, b) {
  return a.eastNm * b.northNm - a.northNm * b.eastNm;
}

function freezePoint(point) {
  return Object.freeze({ eastNm: point.eastNm, northNm: point.northNm });
}

function extractBdpState(bdpResult) {
  if (!bdpResult || typeof bdpResult !== "object") {
    throw new TypeError("bdpResult must be a Bomb Delivery Planner result");
  }
  const canonicalInputs = bdpResult.canonicalInputs ?? bdpResult.inputs;
  const publicResult = bdpResult.public;
  if (!canonicalInputs || !publicResult) {
    throw new TypeError("bdpResult must expose canonicalInputs/inputs and public");
  }

  const angleOffDeg = requireFinite(
    "bdpResult.canonicalInputs.angleOffDeg",
    canonicalInputs.angleOffDeg,
  );
  if (Math.abs(angleOffDeg - BOX_ANGLE_OFF_DEG) > HEADING_ALIGNMENT_TOLERANCE_DEG) {
    throw new RangeError("Wheel-BOX geometry v0.4 requires BDP Angle-Off (Heading) = 90 deg");
  }

  return Object.freeze({
    modelId: bdpResult.model?.id ?? null,
    angleOffDeg,
    rollInRangeNm: requirePositive(
      "bdpResult.public.rollInRangeNm",
      publicResult.rollInRangeNm,
    ),
    baseDistanceNm: requirePositive(
      "BDP Base Distance",
      publicResult.baseDistanceNm,
    ),
    leadAngleDeg: requireFinite(
      "bdpResult.public.leadAngleDeg",
      publicResult.leadAngleDeg,
    ),
    initialSpeedKcas: requirePositive(
      "bdpResult.public.resolvedInitialSpeedKcas",
      publicResult.resolvedInitialSpeedKcas,
    ),
    initialAltitudeMslFt: requireFinite(
      "bdpResult.public.resolvedInitialAltitudeMslFt",
      publicResult.resolvedInitialAltitudeMslFt,
    ),
  });
}

/**
 * Bilateral Wheel-BOX attack-path geometry sourced from one BDP result.
 *
 * RIGHT and LEFT use the same scalar state. LEFT is the exact reflection of
 * RIGHT about the Attack Heading axis. Orbit remains outside this core.
 */
export function calculateWheelBoxGeometryV0_4({
  attackHeadingDeg,
  bdpResult,
  wheelRadiusNm = null,
  baseTurnLoadFactorG = DEFAULT_BASE_TURN_LOAD_FACTOR_G,
  direction = "RIGHT",
}) {
  const resolvedDirection = requireDirection(direction);
  const turnSideSign = resolvedDirection === "RIGHT" ? 1 : -1;
  const attackHeading = normalizeBearingDeg(requireFinite("attackHeadingDeg", attackHeadingDeg));
  const bdp = extractBdpState(bdpResult);
  const minWheelRadiusNm = bdp.baseDistanceNm;
  const selectedWheelRadiusNm = wheelRadiusNm ?? minWheelRadiusNm;
  requirePositive("wheelRadiusNm", selectedWheelRadiusNm);
  if (selectedWheelRadiusNm < minWheelRadiusNm - EPSILON_NM) {
    throw new RangeError("wheelRadiusNm must be >= BDP Min Wheel Radius");
  }
  requireFinite("baseTurnLoadFactorG", baseTurnLoadFactorG);
  if (!(baseTurnLoadFactorG > 1)) {
    throw new RangeError("baseTurnLoadFactorG must be > 1");
  }

  const initialTasKt = casToTas(bdp.initialSpeedKcas, bdp.initialAltitudeMslFt);
  const baseTurnPerformance = coordinatedTurnFromTasAndLoadFactor({
    tasKt: initialTasKt,
    loadFactorG: baseTurnLoadFactorG,
    headingChangeDeg: BASE_TURN_HEADING_CHANGE_DEG,
  });
  const baseTurnRadiusNm = baseTurnPerformance.turnRadiusNm;
  const abeamExtensionDistanceNm = bdp.baseDistanceNm - baseTurnRadiusNm;

  const oa1BearingDeg = normalizeBearingDeg(
    attackHeading + 180 - turnSideSign * bdp.leadAngleDeg,
  );
  const oa1Point = pointFromBearingRange(oa1BearingDeg, bdp.rollInRangeNm);

  const abeamBearingDeg = normalizeBearingDeg(attackHeading + turnSideSign * 90);
  const abeamPoint = pointFromBearingRange(abeamBearingDeg, selectedWheelRadiusNm);
  const abeamExtensionHeadingDeg = normalizeBearingDeg(attackHeading + 180);
  const oa2FromAbeamPoint = translatePoint(
    abeamPoint,
    abeamExtensionHeadingDeg,
    abeamExtensionDistanceNm,
  );
  const iBtAngleDeg =
    (Math.atan2(selectedWheelRadiusNm, abeamExtensionDistanceNm) * 180) / Math.PI;
  const oa2RangeNm = Math.hypot(selectedWheelRadiusNm, abeamExtensionDistanceNm);
  const oa2BearingDeg = normalizeBearingDeg(
    attackHeading + 180 - turnSideSign * iBtAngleDeg,
  );
  const oa2Point = pointFromBearingRange(oa2BearingDeg, oa2RangeNm);
  const oa2ConstructionResidualNm = vectorRangeNm(
    vectorBetween(oa2FromAbeamPoint, oa2Point),
  );

  const baseTurnStartHeadingDeg = abeamExtensionHeadingDeg;
  const baseTurnCenterPoint = translatePoint(
    oa2Point,
    baseTurnStartHeadingDeg + turnSideSign * 90,
    baseTurnRadiusNm,
  );
  const baseTurnEndPoint = translatePoint(
    baseTurnCenterPoint,
    baseTurnStartHeadingDeg,
    baseTurnRadiusNm,
  );
  const baseTurnEndHeadingDeg = normalizeBearingDeg(
    baseTurnStartHeadingDeg + turnSideSign * BASE_TURN_HEADING_CHANGE_DEG,
  );

  const baseLegHeadingDeg = normalizeBearingDeg(
    attackHeading - turnSideSign * BOX_ANGLE_OFF_DEG,
  );
  const baseLegUnit = pointFromBearingRange(baseLegHeadingDeg, 1);
  const endToOa1Vector = vectorBetween(baseTurnEndPoint, oa1Point);
  const signedBaseLegDistanceNm = dot(endToOa1Vector, baseLegUnit);
  const baseLegCrossTrackResidualNm = cross(baseLegUnit, endToOa1Vector);
  if (Math.abs(baseLegCrossTrackResidualNm) > BDP_ALIGNMENT_TOLERANCE_NM) {
    throw new RangeError(
      "BDP Base Distance / Roll-in Range / Lead Angle do not define one Base Leg line",
    );
  }

  const minimumWheelRadiusForBaseLegNm =
    selectedWheelRadiusNm - signedBaseLegDistanceNm;
  const baseLegAvailable = signedBaseLegDistanceNm >= -EPSILON_NM;
  const baseLegDistanceNm = baseLegAvailable ? Math.max(0, signedBaseLegDistanceNm) : null;
  const baseLegTimeSec =
    baseLegDistanceNm === null ? null : baseLegDistanceNm / (initialTasKt / 3600);
  const reconstructedOa1Point =
    baseLegDistanceNm === null
      ? { ...baseTurnEndPoint }
      : translatePoint(baseTurnEndPoint, baseLegHeadingDeg, baseLegDistanceNm);
  const closureVector = vectorBetween(reconstructedOa1Point, oa1Point);
  const closureResidualNm = vectorRangeNm(closureVector);
  const baseLegHeadingResidualDeg = headingResidualDeg(
    baseTurnEndHeadingDeg,
    baseLegHeadingDeg,
  );
  const headingCloses =
    Math.abs(baseLegHeadingResidualDeg) <= HEADING_ALIGNMENT_TOLERANCE_DEG;
  const positionCloses = baseLegAvailable && closureResidualNm <= EPSILON_NM;

  return Object.freeze({
    model: Object.freeze({ ...WHEEL_BOX_GEOMETRY_MODEL_V0_4 }),
    direction: resolvedDirection,
    turnSideSign,
    attackHeadingDeg: attackHeading,
    target: Object.freeze({ eastNm: 0, northNm: 0 }),
    bdp: Object.freeze({ ...bdp }),
    wheel: Object.freeze({
      role: "ATTACK_PATTERN",
      radiusMode: wheelRadiusNm === null ? "MIN_WHEEL_RADIUS" : "MANUAL",
      minWheelRadiusNm,
      radiusNm: selectedWheelRadiusNm,
      minimumRadiusForBaseLegNm: minimumWheelRadiusForBaseLegNm,
    }),
    oa1: Object.freeze({
      rangeNm: bdp.rollInRangeNm,
      bearingDeg: oa1BearingDeg,
      ...freezePoint(oa1Point),
    }),
    abeam: Object.freeze({
      rangeNm: selectedWheelRadiusNm,
      bearingDeg: abeamBearingDeg,
      ...freezePoint(abeamPoint),
      extensionHeadingDeg: abeamExtensionHeadingDeg,
      extensionDistanceNm: abeamExtensionDistanceNm,
      extensionEnd: freezePoint(oa2FromAbeamPoint),
      oa2ConstructionResidualNm,
    }),
    oa2: Object.freeze({
      rangeNm: oa2RangeNm,
      bearingDeg: oa2BearingDeg,
      iBtAngleDeg,
      ...freezePoint(oa2Point),
    }),
    baseTurn: Object.freeze({
      direction: resolvedDirection,
      loadFactorG: baseTurnLoadFactorG,
      bankAngleDeg: baseTurnPerformance.bankAngleDeg,
      radiusNm: baseTurnRadiusNm,
      headingChangeDeg: BASE_TURN_HEADING_CHANGE_DEG,
      turnRateDegSec: baseTurnPerformance.turnRateDegSec,
      arcDistanceNm: baseTurnPerformance.turnDistanceNm,
      turnTimeSec: baseTurnPerformance.turnTimeSec,
      startHeadingDeg: baseTurnStartHeadingDeg,
      endHeadingDeg: baseTurnEndHeadingDeg,
      start: freezePoint(oa2Point),
      center: freezePoint(baseTurnCenterPoint),
      end: freezePoint(baseTurnEndPoint),
    }),
    baseLeg: Object.freeze({
      headingDeg: baseLegHeadingDeg,
      headingResidualDeg: baseLegHeadingResidualDeg,
      signedDistanceNm: signedBaseLegDistanceNm,
      distanceNm: baseLegDistanceNm,
      timeSec: baseLegTimeSec,
      available: baseLegAvailable,
      crossTrackResidualNm: baseLegCrossTrackResidualNm,
      start: freezePoint(baseTurnEndPoint),
      end: freezePoint(reconstructedOa1Point),
    }),
    closure: Object.freeze({
      residualNm: closureResidualNm,
      eastNm: closureVector.eastNm,
      northNm: closureVector.northNm,
      positionCloses,
      headingResidualDeg: baseLegHeadingResidualDeg,
      headingCloses,
      closes: positionCloses && headingCloses,
    }),
  });
}

export const WHEEL_BOX_GEOMETRY_CONSTANTS_V0_4 = Object.freeze({
  BOX_ANGLE_OFF_DEG,
  BASE_TURN_HEADING_CHANGE_DEG,
  DEFAULT_BASE_TURN_LOAD_FACTOR_G,
  EPSILON_NM,
  BDP_ALIGNMENT_TOLERANCE_NM,
  HEADING_ALIGNMENT_TOLERANCE_DEG,
  SUPPORTED_DIRECTIONS,
});
