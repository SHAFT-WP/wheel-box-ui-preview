// V0.5 entrypoints return output truncated to 5 decimals (docs/FE-BE-RULES.md); the *Full
// variants keep full precision for BE-to-BE composition. The V0.4 / V0.3 re-exports below are
// full-precision composition cores, not FE-facing entrypoints.
export {
  WHEEL_BOX_BE_CONSTANTS_V0_5,
  WHEEL_BOX_BE_MODEL_V0_5,
  calculateWheelBoxBeV0_5,
  calculateWheelBoxBeV0_5Full,
  calculateWheelBoxFromBdpResultV0_5,
  calculateWheelBoxFromBdpResultV0_5Full,
  calculateWheelBoxFromBombDeliveryInputV0_5,
  calculateWheelBoxFromBombDeliveryInputV0_5Full,
} from "./wheel-box-be-v0.5.mjs";

export {
  WHEEL_BOX_GEOMETRY_CONSTANTS_V0_4,
  WHEEL_BOX_GEOMETRY_MODEL_V0_4,
  calculateWheelBoxGeometryV0_4,
} from "./wheel-box-geometry-v0.4.mjs";

export {
  WHEEL_RADIUS_MODES_V0_4,
  calculateWheelBoxBeV0_4,
  calculateWheelBoxFromBdpResultV0_4,
  calculateWheelBoxFromBombDeliveryInputV0_4,
} from "./wheel-box-be-v0.4.mjs";

export {
  calculateOrbitHoldingTurnV0_3,
  calculateWheelAndOrbitTurnsV0_3,
  calculateWheelTurnV0_3,
} from "./wheel-orbit-turn-v0.3.mjs";

export const WHEEL_BE_ENTRYPOINT_V0_5 = Object.freeze({
  status: "Work / Pure Calculation / Not Official",
  entrypointVersion: "0.5.1",
  fixedAngleOffDeg: 90,
  supportedDirections: Object.freeze(["RIGHT", "LEFT"]),
  canonicalBdpResultEntrypoint: "calculateWheelBoxFromBdpResultV0_5",
  canonicalBdpInputEntrypoint: "calculateWheelBoxFromBombDeliveryInputV0_5",
  includesUiOrHtml: false,
});
