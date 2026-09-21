import { calculateBombDeliveryV0_3 } from "../bomb-delivery-planner/bomb-delivery-planner-v0.3.mjs";
import { calculateWheelBoxFromBdpResultV0_4 } from "./wheel-box-be-v0.4.mjs";
import { calculateWheelBoxGeometryV0_4 } from "./wheel-box-geometry-v0.4.mjs";

export const WHEEL_BOX_BE_MODEL_V0_5 = Object.freeze({
  id: "wheel-box-be-v0.5-bilateral-bdp-composition",
  version: "0.5.0",
  status: "Work / Pure Calculation / Not Official",
  bombDeliverySource: "bomb-delivery-planner-v0.3-js-facade",
  geometrySource: "wheel-box-bdp-min-wheel-bilateral-closure-v0.4",
  sharedCompositionSource: "wheel-box-be-v0.4-bdp-composition",
});

const BOX_ANGLE_OFF_DEG = 90;
const EPSILON_NM = 1e-12;
const SUPPORTED_DIRECTIONS = Object.freeze(["RIGHT", "LEFT"]);

function requireObject(name, value) {
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    throw new TypeError(`${name} must be an object`);
  }
  return value;
}

function requireDirection(direction) {
  if (!SUPPORTED_DIRECTIONS.includes(direction)) {
    throw new RangeError("direction must be RIGHT or LEFT");
  }
  return direction;
}

function straightSegmentFromGeometry(signedDistanceNm, tasKt) {
  const available = signedDistanceNm >= -EPSILON_NM;
  const distanceNm = available ? Math.max(0, signedDistanceNm) : null;
  return Object.freeze({
    available,
    signedDistanceNm,
    distanceNm,
    timeSec: distanceNm === null ? null : distanceNm / (tasKt / 3600),
  });
}

function composeBilateralResult(shared, geometry) {
  const abeamExtension = straightSegmentFromGeometry(
    geometry.abeam.extensionDistanceNm,
    shared.abeam.abeamExtensionTasKt,
  );
  const baseLeg = straightSegmentFromGeometry(
    geometry.baseLeg.signedDistanceNm,
    shared.baseLeg.baseLegTasKt,
  );
  const issues = [];
  if (!abeamExtension.available) issues.push("NEGATIVE_ABEAM_EXTENSION");
  if (!baseLeg.available) issues.push("WHEEL_RADIUS_REQUIRED_FOR_FORWARD_BASE_LEG");
  if (!geometry.closure.closes && baseLeg.available) issues.push("OA1_CLOSURE_FAILED");

  const wheelRadiusForZeroBaseLegNm = geometry.wheel.minimumRadiusForBaseLegNm;
  const minimumUsableWheelRadiusNm = Math.max(
    shared.wheel.minWheelRadiusNm,
    wheelRadiusForZeroBaseLegNm,
  );

  return Object.freeze({
    ...shared,
    model: Object.freeze({ ...WHEEL_BOX_BE_MODEL_V0_5 }),
    direction: geometry.direction,
    turnSideSign: geometry.turnSideSign,
    attackHeadingDeg: geometry.attackHeadingDeg,
    target: geometry.target,
    wheel: Object.freeze({
      ...shared.wheel,
      minimumUsableWheelRadiusNm,
      wheelRadiusForZeroBaseLegNm,
    }),
    abeam: Object.freeze({
      ...shared.abeam,
      abeamRangeNm: geometry.abeam.rangeNm,
      abeamBearingDeg: geometry.abeam.bearingDeg,
      eastNm: geometry.abeam.eastNm,
      northNm: geometry.abeam.northNm,
      extensionEnd: geometry.abeam.extensionEnd,
      oa2ConstructionResidualNm: geometry.abeam.oa2ConstructionResidualNm,
      abeamExtensionHeadingDeg: geometry.abeam.extensionHeadingDeg,
      abeamExtensionSignedDistanceNm: abeamExtension.signedDistanceNm,
      abeamExtensionAvailable: abeamExtension.available,
      abeamExtensionDistanceNm: abeamExtension.distanceNm,
      abeamExtensionTimeSec: abeamExtension.timeSec,
    }),
    oa1: Object.freeze({
      oa1RangeNm: geometry.oa1.rangeNm,
      oa1BearingDeg: geometry.oa1.bearingDeg,
      eastNm: geometry.oa1.eastNm,
      northNm: geometry.oa1.northNm,
    }),
    oa2: Object.freeze({
      oa2RangeNm: geometry.oa2.rangeNm,
      oa2BearingDeg: geometry.oa2.bearingDeg,
      ibtAngleDeg: geometry.oa2.iBtAngleDeg,
      eastNm: geometry.oa2.eastNm,
      northNm: geometry.oa2.northNm,
    }),
    baseTurn: Object.freeze({
      ...shared.baseTurn,
      baseTurnDirection: geometry.baseTurn.direction,
      baseTurnG: geometry.baseTurn.loadFactorG,
      baseTurnBankAngleDeg: geometry.baseTurn.bankAngleDeg,
      baseTurnRadiusNm: geometry.baseTurn.radiusNm,
      baseTurnHeadingChangeDeg: geometry.baseTurn.headingChangeDeg,
      baseTurnRateDegSec: geometry.baseTurn.turnRateDegSec,
      baseTurnArcDistanceNm: geometry.baseTurn.arcDistanceNm,
      baseTurnTimeSec: geometry.baseTurn.turnTimeSec,
      baseTurnStartHeadingDeg: geometry.baseTurn.startHeadingDeg,
      baseTurnEndHeadingDeg: geometry.baseTurn.endHeadingDeg,
      start: geometry.baseTurn.start,
      center: geometry.baseTurn.center,
      end: geometry.baseTurn.end,
    }),
    baseLeg: Object.freeze({
      ...shared.baseLeg,
      baseLegHeadingDeg: geometry.baseLeg.headingDeg,
      baseLegHeadingResidualDeg: geometry.baseLeg.headingResidualDeg,
      baseLegAvailable: baseLeg.available,
      baseLegSignedDistanceNm: baseLeg.signedDistanceNm,
      baseLegDistanceNm: baseLeg.distanceNm,
      baseLegTimeSec: baseLeg.timeSec,
      baseLegCrossTrackResidualNm: geometry.baseLeg.crossTrackResidualNm,
      start: geometry.baseLeg.start,
      end: geometry.baseLeg.end,
    }),
    closure: Object.freeze({ ...geometry.closure }),
    status: Object.freeze({
      geometryCloses: geometry.closure.closes,
      abeamExtensionAvailable: abeamExtension.available,
      baseLegAvailable: baseLeg.available,
      attackPathAvailable:
        geometry.closure.closes && abeamExtension.available && baseLeg.available,
      issues: Object.freeze(issues),
    }),
  });
}

/**
 * Current bilateral Wheel-BOX BE composition.
 *
 * Shared BDP/radius/performance validation stays in V0.4. V0.5 replaces only
 * the direction-limited geometry with V0.4 bilateral RIGHT/LEFT geometry.
 */
export function calculateWheelBoxFromBdpResultV0_5(input) {
  requireObject("input", input);
  const direction = requireDirection(input.direction ?? "RIGHT");

  const shared = calculateWheelBoxFromBdpResultV0_4({
    ...input,
    direction: "RIGHT",
  });
  const geometry = calculateWheelBoxGeometryV0_4({
    attackHeadingDeg: shared.attackHeadingDeg,
    bdpResult: input.bdpResult,
    wheelRadiusNm:
      shared.wheel.wheelRadiusMode === "MIN_WHEEL_RADIUS"
        ? null
        : shared.wheel.wheelRadiusNm,
    baseTurnLoadFactorG: shared.baseTurn.baseTurnG,
    direction,
  });

  return composeBilateralResult(shared, geometry);
}

export function calculateWheelBoxFromBombDeliveryInputV0_5({
  bombDeliveryInput,
  ...wheelBoxInput
}) {
  requireObject("bombDeliveryInput", bombDeliveryInput);
  const bdpResult = calculateBombDeliveryV0_3({
    ...bombDeliveryInput,
    angleOffDeg: BOX_ANGLE_OFF_DEG,
  });
  return calculateWheelBoxFromBdpResultV0_5({
    ...wheelBoxInput,
    bdpResult,
  });
}

export const calculateWheelBoxBeV0_5 = calculateWheelBoxFromBdpResultV0_5;

export const WHEEL_BOX_BE_CONSTANTS_V0_5 = Object.freeze({
  BOX_ANGLE_OFF_DEG,
  EPSILON_NM,
  SUPPORTED_DIRECTIONS,
});
