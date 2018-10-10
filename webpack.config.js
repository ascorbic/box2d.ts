const path = require("path");
module.exports = {
    devtool: "inline-source-map",
    entry: "./Testbed/boot.ts",
    mode: "development",
    output: {
        pathinfo: false
    },
    optimization: {
        removeAvailableModules: false,
        removeEmptyChunks: false,
        splitChunks: false
    },
    devServer: {
        contentBase: path.join(__dirname, "Testbed")
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
    }
    // plugins: [new HtmlWebpackPlugin()]
};
