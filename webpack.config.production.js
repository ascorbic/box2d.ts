const DeclarationBundlerPlugin = require("declaration-bundler-webpack-plugin");
const path = require("path");
module.exports = {
    entry: "./Box2D/Box2D.ts",
    mode: "production",
    output: {
        path: path.join(__dirname, "/dist"),
        filename: "box2d.js",
        library: "box2d",
        libraryTarget: "umd",
        umdNamedDefine: true
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
                        transpileOnly: true
                    }
                }
            }
        ]
    },
    resolve: {
        extensions: [".ts", ".tsx", ".js"],
        alias: {
            Box2D: path.resolve(__dirname, "Box2D/Box2D.ts")
        }
    },
    plugins: [
        new DeclarationBundlerPlugin({
            moduleName: "box2d",
            out: "./dist/box2d.d.ts"
        })
    ]
};
