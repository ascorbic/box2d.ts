import b2 from "./liquidfun.js";
import b2wasm from "./liquidfun.wasm";
export const init = () => {
    return b2({
        locateFile: (path: string) => {
            console.log("locate: " + path);
            if (path.endsWith(".wasm")) {
                return b2wasm;
            }
            return path;
        },
    });
};
