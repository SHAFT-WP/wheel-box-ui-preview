import { casToTas } from "../../../common/airspeed/airspeed-v0.1.mjs";
import { calculateBombDeliveryV0_3 } from "../bomb-delivery-planner/bomb-delivery-planner-v0.3.mjs";
import { calculateWheelBoxGeometryV0_3 } from "./wheel-box-geometry-v0.3.mjs";
import {
  calculateOrbitHoldingTurnV0_3,
  calculateWheelTurnV0_3,
} from "./wheel-orbit-turn-v0.3.mjs";

export const WHEEL_BOX_BE_MODEL_V0_4 = Object.freeze({
  id: "wheel-box-be-v0.4-bdp-composition",
  version: "0.4.0",
  status: "Work / Pure Calculation / Not Official",
  bombDeliverySource: "bomb-delivery-planner-v0.3-js-facade",
  geometrySource: "wheel-box-bdp-min-wheel-closure-v0.3",
});

export const WHEEL_RADIUS_MODES_V0_4 = Object.freeze({
  MIN_WHEEL_RADIUS: "MIN_WHEEL_RADIUS",
  MANUAL: "MANUAL",
});

const BOX_ANGLE_OFF_DEG = 90;
const DEFAULT_BASE_TURN_G = 2;
const FT_PER_NM = 6076.11549;
const EPSILON_NM = 1e-12;
const BDP_ALIGNMENT_TOLERANCE_NM = 1e-9;

function requireObject(name, value) {
  if (!value || typeof value !== "object" || Array.isArray(value)) {
    throw new TypeError(`${name} must be an object`);
  }
  return value;
}

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

function extractBdpSource(bdpResult) {
  requireObject("bdpResult", bdpResult);
  const canonicalInputs = bdpResult.canonicalInputs ?? bdpResult.inputs;
  const publicResult = bdpResult.public;
  requireObject("bdpResult.canonicalInputs/inputs", canonicalInputs);
  requireObject("bdpResult.public", publicResult);

  const angleOffDeg = requireFinite(
    "bdpResult.canonicalInputs.angleOffDeg",
    canonicalInputs.angleOffDeg,
  );
  if (Math.abs(angleOffDeg - BOX_ANGLE_OFF_DEG) > EPSILON_NM) {
    throw new RangeError("Wheel-BOX BE v0.4 requires BDP Angle-Off (Heading) = 90 deg");
  }


  return Object.freeze({
    modelId: bdpResult.model?.id ?? null,
    angleOffDeg,
    targetElevationMslFt: requireFinite(
      "bdpResult.canonicalInputs.targetElevationMslFt",
      canonicalInputs.targetElevationMslFt,
    ),
    initialSpeedKcas: requirePositive(
      "bdpResult.public.resolvedInitialSpeedKcas",
      publicResult.resolvedInitialSpeedKcas,
    ),
    initialAltitudeMslFt: requireFinite(
      "bdpResult.public.resolvedInitialAltitudeMslFt",
      publicResult.resolvedInitialAltitudeMslFt,
    ),
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
  });
}

function resolveSpeedAltitude({
  prefix,
  initialSpeedKcas,
  initialAltitudeMslFt,
  speedKcas,
  altitudeMslFt,
}) {
  const resolvedSpeedKcas = speedKcas ?? initialSpeedKcas;
  const resolvedAltitudeMslFt = altitudeMslFt ?? initialAltitudeMslFt;
  requirePositive(`${prefix}SpeedKcas`, resolvedSpeedKcas);
  requireFinite(`${prefix}AltitudeMslFt`, resolvedAltitudeMslFt);
  return Object.freeze({
    speedKcas: resolvedSpeedKcas,
    altitudeMslFt: resolvedAltitudeMslFt,
    tasKt: casToTas(resolvedSpeedKcas, resolvedAltitudeMslFt),
  });
}

function resolveWheelRadiusControl({
  wheelRadiusMode,
  wheelRadiusNm,
  minWheelRadiusNm,
}) {
  const mode =
    wheelRadiusMode ??
    (wheelRadiusNm === null || wheelRadiusNm === undefined
      ? WHEEL_RADIUS_MODES_V0_4.MIN_WHEEL_RADIUS
      : WHEEL_RADIUS_MODES_V0_4.MANUAL);

  if (!Object.values(WHEEL_RADIUS_MODES_V0_4).includes(mode)) {
    throw new RangeError("wheelRadiusMode must be MIN_WHEEL_RADIUS or MANUAL");
  }

  if (mode === WHEEL_RADIUS_MODES_V0_4.MIN_WHEEL_RADIUS) {
    if (wheelRadiusNm !== null && wheelRadiusNm !== undefined) {
      requirePositive("wheelRadiusNm", wheelRadiusNm);
      if (Math.abs(wheelRadiusNm - minWheelRadiusNm) > EPSILON_NM) {
        throw new RangeError(
          "wheelRadiusNm must be omitted or equal Min Wheel Radius in MIN_WHEEL_RADIUS mode",
        );
      }
    }
    return Object.freeze({
      mode,
      selectedRadiusNm: minWheelRadiusNm,
      geometryRadiusInputNm: null,
    });
  }

  const selectedRadiusNm = requirePositive("wheelRadiusNm", wheelRadiusNm);
  if (selectedRadiusNm < minWheelRadiusNm - EPSILON_NM) {
    throw new RangeError("wheelRadiusNm must be >= BDP Min Wheel Radius");
  }
  return Object.freeze({ mode, selectedRadiusNm, geometryRadiusInputNm: selectedRadiusNm });
}

function resolveBaseTurnG(baseTurnG, baseTurnLoadFactorG) {
  if (baseTurnG !== null && baseTurnG !== undefined) {
    requireFinite("baseTurnG", baseTurnG);
  }
  if (baseTurnLoadFactorG !== null && baseTurnLoadFactorG !== undefined) {
    requireFinite("baseTurnLoadFactorG", baseTurnLoadFactorG);
  }
  if (
    baseTurnG !== null &&
    baseTurnG !== undefined &&
    baseTurnLoadFactorG !== null &&
    baseTurnLoadFactorG !== undefined &&
    Math.abs(baseTurnG - baseTurnLoadFactorG) > EPSILON_NM
  ) {
    throw new RangeError("baseTurnG and baseTurnLoadFactorG must be equal when both are provided");
  }
  const selected = baseTurnG ?? baseTurnLoadFactorG ?? DEFAULT_BASE_TURN_G;
  if (!(selected > 1)) throw new RangeError("baseTurnG must be > 1");
  return selected;
}

function slantRangeNm(horizontalRangeNm, altitudeMslFt, targetElevationMslFt) {
  return Math.hypot(
    horizontalRangeNm,
    (altitudeMslFt - targetElevationMslFt) / FT_PER_NM,
  );
}

function operationalStraightSegment(signedDistanceNm, state) {
  requireFinite("signedDistanceNm", signedDistanceNm);
  const available = signedDistanceNm >= -EPSILON_NM;
  const distanceNm = available ? Math.max(0, signedDistanceNm) : null;
  return Object.freeze({
    available,
    signedDistanceNm,
    distanceNm,
    timeSec: distanceNm === null ? null : distanceNm / (state.tasKt / 3600),
  });
}

function buildOrbit({
  source,
  orbitRadiusNm,
  orbitSpeedKcas,
  orbitAltitudeMslFt,
}) {
  if (orbitRadiusNm === null || orbitRadiusNm === undefined) {
    if (orbitSpeedKcas !== null && orbitSpeedKcas !== undefined) {
      throw new TypeError("orbitRadiusNm is required when orbitSpeedKcas is provided");
    }
    if (orbitAltitudeMslFt !== null && orbitAltitudeMslFt !== undefined) {
      throw new TypeError("orbitRadiusNm is required when orbitAltitudeMslFt is provided");
    }
    return null;
  }

  const orbit = calculateOrbitHoldingTurnV0_3({
    initialSpeedKcas: source.initialSpeedKcas,
    initialAltitudeMslFt: source.initialAltitudeMslFt,
    orbitRadiusNm,
    orbitSpeedKcas,
    orbitAltitudeMslFt,
  });
  return Object.freeze({
    ...orbit,
    orbitSlantRangeNm: slantRangeNm(
      orbit.orbitRadiusNm,
      orbit.orbitAltitudeMslFt,
      source.targetElevationMslFt,
    ),
  });
}

/**
 * Canonical Wheel-BOX BE composition from one authoritative BDP result.
 *
 * BDP owns the bombing state. Wheel-BOX owns Wheel radius selection, the
 * attack-path geometry and segment performance. Orbit is optional and remains
 * a separate waiting/deconfliction/targeting calculation.
 */
export function calculateWheelBoxFromBdpResultV0_4({
  bdpResult,
  attackHeadingDeg,
  direction = "RIGHT",
  wheelRadiusMode = null,
  wheelRadiusNm = null,
  wheelSpeedKcas = null,
  wheelAltitudeMslFt = null,
  baseTurnG = null,
  baseTurnLoadFactorG = null,
  abeamExtensionSpeedKcas = null,
  abeamExtensionAltitudeMslFt = null,
  baseLegSpeedKcas = null,
  baseLegAltitudeMslFt = null,
  orbitRadiusNm = null,
  orbitSpeedKcas = null,
  orbitAltitudeMslFt = null,
}) {
  const source = extractBdpSource(bdpResult);
  const selectedBaseTurnG = resolveBaseTurnG(baseTurnG, baseTurnLoadFactorG);
  const radiusControl = resolveWheelRadiusControl({
    wheelRadiusMode,
    wheelRadiusNm,
    minWheelRadiusNm: source.baseDistanceNm,
  });

  const geometry = calculateWheelBoxGeometryV0_3({
    attackHeadingDeg,
    bdpResult,
    wheelRadiusNm: radiusControl.geometryRadiusInputNm,
    baseTurnLoadFactorG: selectedBaseTurnG,
    direction,
  });
  const wheelTurn = calculateWheelTurnV0_3({
    initialSpeedKcas: source.initialSpeedKcas,
    initialAltitudeMslFt: source.initialAltitudeMslFt,
    minWheelRadiusNm: source.baseDistanceNm,
    wheelRadiusNm: radiusControl.geometryRadiusInputNm,
    wheelSpeedKcas,
    wheelAltitudeMslFt,
  });

  const abeamExtensionState = resolveSpeedAltitude({
    prefix: "abeamExtension",
    initialSpeedKcas: source.initialSpeedKcas,
    initialAltitudeMslFt: source.initialAltitudeMslFt,
    speedKcas: abeamExtensionSpeedKcas,
    altitudeMslFt: abeamExtensionAltitudeMslFt,
  });
  const baseLegState = resolveSpeedAltitude({
    prefix: "baseLeg",
    initialSpeedKcas: source.initialSpeedKcas,
    initialAltitudeMslFt: source.initialAltitudeMslFt,
    speedKcas: baseLegSpeedKcas,
    altitudeMslFt: baseLegAltitudeMslFt,
  });
  const baseTurnTasKt = casToTas(source.initialSpeedKcas, source.initialAltitudeMslFt);

  const abeamExtension = operationalStraightSegment(
    geometry.abeam.extensionDistanceNm,
    abeamExtensionState,
  );
  const baseLeg = operationalStraightSegment(
    geometry.baseLeg.signedDistanceNm,
    baseLegState,
  );

  const issues = [];
  if (!abeamExtension.available) issues.push("NEGATIVE_ABEAM_EXTENSION");
  if (!baseLeg.available) issues.push("WHEEL_RADIUS_REQUIRED_FOR_FORWARD_BASE_LEG");
  if (!geometry.closure.closes && baseLeg.available) issues.push("OA1_CLOSURE_FAILED");

  const wheelRadiusForZeroBaseLegNm = geometry.wheel.minimumRadiusForBaseLegNm;
  const minimumUsableWheelRadiusNm = Math.max(
    source.baseDistanceNm,
    wheelRadiusForZeroBaseLegNm,
  );
  const orbit = buildOrbit({
    source,
    orbitRadiusNm,
    orbitSpeedKcas,
    orbitAltitudeMslFt,
  });

  return Object.freeze({
    model: Object.freeze({ ...WHEEL_BOX_BE_MODEL_V0_4 }),
    bombDelivery: bdpResult,
    bdp: source,
    direction: geometry.direction,
    attackHeadingDeg: geometry.attackHeadingDeg,
    target: geometry.target,
    wheel: Object.freeze({
      role: "ATTACK_PATTERN",
      wheelRadiusMode: radiusControl.mode,
      minWheelRadiusNm: source.baseDistanceNm,
      wheelRadiusNm: radiusControl.selectedRadiusNm,
      minimumUsableWheelRadiusNm,
      wheelRadiusForZeroBaseLegNm,
      wheelSpeedKcas: wheelTurn.wheelSpeedKcas,
      wheelAltitudeMslFt: wheelTurn.wheelAltitudeMslFt,
      wheelBankAngleDeg: wheelTurn.wheelBankAngleDeg,
      wheelTurn180TimeSec: wheelTurn.wheelTurn180TimeSec,
      wheelSlantRangeNm: slantRangeNm(
        radiusControl.selectedRadiusNm,
        wheelTurn.wheelAltitudeMslFt,
        source.targetElevationMslFt,
      ),
    }),
    abeam: Object.freeze({
      abeamRangeNm: geometry.abeam.rangeNm,
      abeamBearingDeg: geometry.abeam.bearingDeg,
      eastNm: geometry.abeam.eastNm,
      northNm: geometry.abeam.northNm,
      extensionEnd: geometry.abeam.extensionEnd,
      oa2ConstructionResidualNm: geometry.abeam.oa2ConstructionResidualNm,
      abeamExtensionHeadingDeg: geometry.abeam.extensionHeadingDeg,
      abeamExtensionSpeedKcas: abeamExtensionState.speedKcas,
      abeamExtensionAltitudeMslFt: abeamExtensionState.altitudeMslFt,
      abeamExtensionTasKt: abeamExtensionState.tasKt,
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
      baseTurnSpeedKcas: source.initialSpeedKcas,
      baseTurnAltitudeMslFt: source.initialAltitudeMslFt,
      baseTurnTasKt,
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
      baseLegHeadingDeg: geometry.baseLeg.headingDeg,
      baseLegSpeedKcas: baseLegState.speedKcas,
      baseLegAltitudeMslFt: baseLegState.altitudeMslFt,
      baseLegTasKt: baseLegState.tasKt,
      baseLegAvailable: baseLeg.available,
      baseLegSignedDistanceNm: baseLeg.signedDistanceNm,
      baseLegDistanceNm: baseLeg.distanceNm,
      baseLegTimeSec: baseLeg.timeSec,
      baseLegCrossTrackResidualNm: geometry.baseLeg.crossTrackResidualNm,
      start: geometry.baseLeg.start,
      end: geometry.baseLeg.end,
    }),
    closure: Object.freeze({ ...geometry.closure }),
    orbit,
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
 * Convenience entrypoint for callers that own canonical BDP input rather than
 * an already-calculated result. Wheel-BOX fixes Angle-Off (Heading) to 90 deg.
 */
export function calculateWheelBoxFromBombDeliveryInputV0_4({
  bombDeliveryInput,
  ...wheelBoxInput
}) {
  requireObject("bombDeliveryInput", bombDeliveryInput);
  const bdpResult = calculateBombDeliveryV0_3({
    ...bombDeliveryInput,
    angleOffDeg: BOX_ANGLE_OFF_DEG,
  });
  return calculateWheelBoxFromBdpResultV0_4({
    ...wheelBoxInput,
    bdpResult,
  });
}

export const calculateWheelBoxBeV0_4 = calculateWheelBoxFromBdpResultV0_4;

export const WHEEL_BOX_BE_CONSTANTS_V0_4 = Object.freeze({
  BOX_ANGLE_OFF_DEG,
  DEFAULT_BASE_TURN_G,
  FT_PER_NM,
  EPSILON_NM,
  BDP_ALIGNMENT_TOLERANCE_NM,
});
