type ThreePrimitiveProps = Record<string, unknown>;

type PalletizerThreeElements = {
  ambientLight: ThreePrimitiveProps;
  axesHelper: ThreePrimitiveProps;
  boxGeometry: ThreePrimitiveProps;
  color: ThreePrimitiveProps;
  cylinderGeometry: ThreePrimitiveProps;
  directionalLight: ThreePrimitiveProps;
  gridHelper: ThreePrimitiveProps;
  group: ThreePrimitiveProps;
  mesh: ThreePrimitiveProps;
  meshBasicMaterial: ThreePrimitiveProps;
  meshStandardMaterial: ThreePrimitiveProps;
  planeGeometry: ThreePrimitiveProps;
  sphereGeometry: ThreePrimitiveProps;
};

declare global {
  namespace JSX {
    interface IntrinsicElements extends PalletizerThreeElements {}
  }
}

declare module "react/jsx-runtime" {
  namespace JSX {
    interface IntrinsicElements extends PalletizerThreeElements {}
  }
}

export {};
