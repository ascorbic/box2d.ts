import {
    b2World,
    b2Body,
    b2BodyDef,
    b2FixtureDef,
    b2PolygonShape,
    b2CircleShape,
    b2Vec2,
    XY,
    b2BodyType,
} from "Box2D";

type ShapeDef = PolygonDef | CircleDef;
type PolygonDef = [
    number,
    number,
    number,
    number,
    number,
    number,
    boolean,
    "POLYGON",
    b2Vec2[][]
];

type CircleDef = [
    number,
    number,
    number,
    number,
    number,
    number,
    boolean,
    "CIRCLE",
    XY,
    number
];

export class PhysicsData {
    // ptm ratio
    public ptm_ratio: number = 320;

    // the physcis data
    public dict: {
        [key: string]: ShapeDef[];
    } = {};

    //
    // bodytype:
    //  b2_staticBody
    //  b2_kinematicBody
    //  b2_dynamicBody

    public createBody(
        name: string,
        world: b2World,
        bodyType: b2BodyType,
        userData: any = null,
    ): b2Body {
        const fixtures = this.dict[name];

        let body: b2Body;

        // prepare body def
        const bodyDef: b2BodyDef = new b2BodyDef();
        bodyDef.type = bodyType;
        bodyDef.userData = userData;

        // create the body
        body = world.CreateBody(bodyDef);

        // prepare fixtures
        fixtures.forEach((fixture) => {
            const fixtureDef: b2FixtureDef = new b2FixtureDef();

            fixtureDef.density = fixture[0];
            fixtureDef.friction = fixture[1];
            fixtureDef.restitution = fixture[2];

            fixtureDef.filter.categoryBits = fixture[3];
            fixtureDef.filter.maskBits = fixture[4];
            fixtureDef.filter.groupIndex = fixture[5];
            fixtureDef.isSensor = fixture[6];

            if (fixture[7] === "POLYGON") {
                fixture[8].forEach((polygon) => {
                    const polygonShape: b2PolygonShape = new b2PolygonShape();
                    polygonShape.SetAsArray(polygon, polygon.length);
                    fixtureDef.shape = polygonShape;

                    body.CreateFixture(fixtureDef);
                });
            } else if (fixture[7] === "CIRCLE") {
                const circleShape: b2CircleShape = new b2CircleShape(
                    fixture[9],
                );
                circleShape.Set(fixture[8]);
                fixtureDef.shape = circleShape;
                body.CreateFixture(fixtureDef);
            }
        });

        return body;
    }

    public constructor() {
        const ptm = this.ptm_ratio;
        this.dict.bone = [
            [
                // density, friction, restitution
                2,
                0,
                0,
                // categoryBits, maskBits, groupIndex, isSensor
                1,
                65535,
                0,
                false,
                "POLYGON",

                // vertexes of decomposed polygons
                [
                    [
                        new b2Vec2(28 / ptm, 120.5 / ptm),
                        new b2Vec2(22.5 / ptm, 107 / ptm),
                        new b2Vec2(68.5 / ptm, 93 / ptm),
                        new b2Vec2(70.5 / ptm, 100 / ptm),
                        new b2Vec2(70.5 / ptm, 107 / ptm),
                        new b2Vec2(59 / ptm, 124.5 / ptm),
                        new b2Vec2(52 / ptm, 127.5 / ptm),
                        new b2Vec2(40 / ptm, 127.5 / ptm),
                    ],
                    [
                        new b2Vec2(127.5 / ptm, 52 / ptm),
                        new b2Vec2(122.5 / ptm, 62 / ptm),
                        new b2Vec2(114 / ptm, 68.5 / ptm),
                        new b2Vec2(89 / ptm, 66.5 / ptm),
                        new b2Vec2(106.5 / ptm, 22 / ptm),
                        new b2Vec2(114 / ptm, 23.5 / ptm),
                        new b2Vec2(122 / ptm, 29.5 / ptm),
                        new b2Vec2(127.5 / ptm, 40 / ptm),
                    ],
                    [
                        new b2Vec2(-0.5 / ptm, 75 / ptm),
                        new b2Vec2(4.5 / ptm, 65 / ptm),
                        new b2Vec2(13 / ptm, 58.5 / ptm),
                        new b2Vec2(38 / ptm, 60.5 / ptm),
                        new b2Vec2(20.5 / ptm, 105 / ptm),
                        new b2Vec2(13 / ptm, 103.5 / ptm),
                        new b2Vec2(5 / ptm, 97.5 / ptm),
                        new b2Vec2(-0.5 / ptm, 87 / ptm),
                    ],
                    [
                        new b2Vec2(89 / ptm, 66.5 / ptm),
                        new b2Vec2(114 / ptm, 68.5 / ptm),
                        new b2Vec2(100 / ptm, 70.5 / ptm),
                    ],
                    [
                        new b2Vec2(38 / ptm, 60.5 / ptm),
                        new b2Vec2(13 / ptm, 58.5 / ptm),
                        new b2Vec2(27 / ptm, 56.5 / ptm),
                    ],
                    [
                        new b2Vec2(104.5 / ptm, 20 / ptm),
                        new b2Vec2(67 / ptm, 88.5 / ptm),
                        new b2Vec2(60 / ptm, 38.5 / ptm),
                        new b2Vec2(60.5 / ptm, 36 / ptm),
                        new b2Vec2(77 / ptm, -0.5 / ptm),
                        new b2Vec2(94 / ptm, 2.5 / ptm),
                        new b2Vec2(99 / ptm, 6.5 / ptm),
                        new b2Vec2(103.5 / ptm, 13 / ptm),
                    ],
                    [
                        new b2Vec2(22.5 / ptm, 107 / ptm),
                        new b2Vec2(28 / ptm, 120.5 / ptm),
                        new b2Vec2(23.5 / ptm, 114 / ptm),
                    ],
                    [
                        new b2Vec2(68 / ptm, 2.5 / ptm),
                        new b2Vec2(76 / ptm, -0.5 / ptm),
                        new b2Vec2(77 / ptm, -0.5 / ptm),
                        new b2Vec2(60.5 / ptm, 36 / ptm),
                        new b2Vec2(58.5 / ptm, 34 / ptm),
                        new b2Vec2(56.5 / ptm, 27 / ptm),
                        new b2Vec2(56.5 / ptm, 20 / ptm),
                        new b2Vec2(59.5 / ptm, 11 / ptm),
                    ],
                    [
                        new b2Vec2(94 / ptm, 2.5 / ptm),
                        new b2Vec2(77 / ptm, -0.5 / ptm),
                        new b2Vec2(87 / ptm, -0.5 / ptm),
                    ],
                    [
                        new b2Vec2(64.5 / ptm, 120 / ptm),
                        new b2Vec2(59 / ptm, 124.5 / ptm),
                        new b2Vec2(70.5 / ptm, 107 / ptm),
                        new b2Vec2(68.5 / ptm, 114 / ptm),
                    ],
                    [
                        new b2Vec2(67 / ptm, 88.5 / ptm),
                        new b2Vec2(104.5 / ptm, 20 / ptm),
                        new b2Vec2(106.5 / ptm, 22 / ptm),
                        new b2Vec2(89 / ptm, 66.5 / ptm),
                    ],
                    [
                        new b2Vec2(60 / ptm, 38.5 / ptm),
                        new b2Vec2(67 / ptm, 88.5 / ptm),
                        new b2Vec2(66.5 / ptm, 91 / ptm),
                        new b2Vec2(22.5 / ptm, 107 / ptm),
                        new b2Vec2(20.5 / ptm, 105 / ptm),
                        new b2Vec2(38 / ptm, 60.5 / ptm),
                    ],
                    [
                        new b2Vec2(22.5 / ptm, 107 / ptm),
                        new b2Vec2(66.5 / ptm, 91 / ptm),
                        new b2Vec2(68.5 / ptm, 93 / ptm),
                    ],
                ],
            ],
        ];
    }
}
