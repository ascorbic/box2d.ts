const path = require("path");
const HtmlWebpackPlugin = require("html-webpack-plugin");
module.exports = {
    devtool: "inline-source-map",
    entry: "./Testbed/boot.ts",
    mode: "production",
    output: {
        path: path.join(__dirname, "/dist")
    },
    optimization: {
        removeAvailableModules: false,
        removeEmptyChunks: false,
        splitChunks: false
    },
    module: {
        rules: [
            {
                test: /\.tsx?$/,
                use: {
                    loader: "ts-loader",
                    options: {
                        transpileOnly: true,
                        experimentalWatchApi: true
                    }
                }
            }
        ]
    },
    resolve: {
        extensions: [".ts", ".tsx", ".js"],
        alias: {
            Box2D: path.resolve(__dirname, "Box2D/Box2D.ts"),
            Testbed: path.resolve(__dirname, "Testbed/Testbed.ts")
        }
    },
    plugins: [
        new HtmlWebpackPlugin({
            title: "Physics demo",
            hash: true
        })
    ]
};
