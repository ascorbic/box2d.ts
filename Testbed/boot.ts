import { Main } from "./Testbed";
let app: Main;
const init = (time: number) => {
    app = new Main(time);
    window.requestAnimationFrame(loop);
};
const loop = (time: number) => {
    window.requestAnimationFrame(loop);
    app.SimulationLoop(time);
};
window.requestAnimationFrame(init);
