import { Box2D } from "./box2d";

declare function Module(
    emscriptenArgs: any
): {
    then: (callback: (module: Box2D) => void) => void;
};
export default Module;
declare module "*.wasm";
