/*
* Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

import * as box2d from "Box2D";
import * as testbed from "Testbed";

export class Hoover extends testbed.Test {
    public static readonly count = 20;

    public sensor: box2d.b2Fixture;
    public teleport: box2d.b2Fixture;
    public bodies: box2d.b2Body[];
    public toTeleport: Set<box2d.b2Body> = new Set<box2d.b2Body>();
    public m_ropeDef = new box2d.b2RopeJointDef();
    public m_rope: box2d.b2RopeJoint | null = null;

    public touching: boolean[][];

    constructor() {
        super();
        /*const int32*/
        const N = 80;
        /*box2d.b2Vec2[]*/
        const vertices = box2d.b2Vec2.MakeArray(N);
        /*float32[]*/
        const masses = box2d.b2MakeNumberArray(N);

        for (let i = 0; i < N; ++i) {
            vertices[i].Set(0.0, 0.0 - 0.25 * i);
            masses[i] = 1.0;
        }
        masses[0] = 0.0;
        masses[1] = 0.0;

        /*box2d.b2RopeDef*/
        // const def = new box2d.b2RopeDef();
        // def.vertices = vertices;
        // def.count = N;
        // def.gravity.Set(0.0, -10.0);
        // def.masses = masses;
        // def.damping = 0.1;
        // def.k2 = 1.0;
        // def.k3 = 0.5;

        this.bodies = new Array(Hoover.count);
        this.touching = new Array(Hoover.count);
        for (let i = 0; i < Hoover.count; ++i) {
            this.touching[i] = new Array(1);
        }

        const bd = new box2d.b2BodyDef();
        const ground = this.m_world.CreateBody(bd);

        {
            const shape = new box2d.b2EdgeShape();
            shape.Set(
                new box2d.b2Vec2(-50.0, 0.0),
                new box2d.b2Vec2(50.0, 0.0),
            );
            ground.CreateFixture(shape, 0.0);
        }

        /*
    {
      const sd = new box2d.b2FixtureDef();
      sd.SetAsBox(10.0, 2.0, new box2d.b2Vec2(0.0, 20.0), 0.0);
      sd.isSensor = true;
      this.m_sensor = ground.CreateFixture(sd);
    }
    */
        let sensor;
        {
            const shape = new box2d.b2CircleShape();
            shape.m_radius = 5.0;
            shape.m_p.Set(0, 30);

            const inner = new box2d.b2CircleShape();
            inner.m_radius = 1.0;
            inner.m_p.Set(0, 30);

            const bd = new box2d.b2BodyDef();
            bd.type = box2d.b2BodyType.b2_dynamicBody;
            const fd = new box2d.b2FixtureDef();
            fd.shape = shape;
            fd.isSensor = true;
            fd.filter.categoryBits = 0x0002;

            const fd2 = new box2d.b2FixtureDef();
            fd2.filter.categoryBits = 0x0002;
            fd2.shape = inner;
            fd2.isSensor = true;
            sensor = this.m_world.CreateBody(bd);
            sensor.SetGravityScale(0);
            this.sensor = sensor.CreateFixture(fd);
            this.teleport = sensor.CreateFixture(fd2);
        }

        {
            const shape = new box2d.b2CircleShape();
            shape.m_radius = 1.0;

            for (let i = 0; i < Hoover.count; ++i) {
                //const bd = new box2d.b2BodyDef();
                bd.type = box2d.b2BodyType.b2_dynamicBody;
                bd.position.Set(-10.0 + 3.0 * i, 20.0);
                bd.userData = this.touching[i];

                this.touching[i][0] = false;
                this.bodies[i] = this.m_world.CreateBody(bd);

                this.bodies[i].CreateFixture(shape, 1.0);
            }
        }

        {
            // /*box2d.b2PolygonShape*/
            // const shape = new box2d.b2PolygonShape();
            // shape.SetAsBox(0.5, 0.25);
            // /*box2d.b2FixtureDef*/
            // const fd = new box2d.b2FixtureDef();
            // fd.shape = shape;
            // fd.density = 10.0;
            // fd.friction = 0.2;
            // fd.filter.categoryBits = 0x0001;
            // fd.filter.maskBits = 0xffff & ~0x0002;
            // /*box2d.b2RevoluteJointDef*/
            // const jd = new box2d.b2RevoluteJointDef();
            // jd.collideConnected = false;
            // /*const int32*/
            // const N = 40;
            // /*const float32*/
            // const y = 30.0;
            // this.m_ropeDef.localAnchorA.Set(0.0, y);
            // /*box2d.b2Body*/
            // let prevBody = ground;
            // for (/*int32*/ let i = 0; i < N; ++i) {
            //     /*box2d.b2BodyDef*/
            //     const bd = new box2d.b2BodyDef();
            //     bd.type = box2d.b2BodyType.b2_dynamicBody;
            //     bd.position.Set(0.5 + 1.0 * i, y);
            //     // if (i === N - 1) {
            //     //     shape.SetAsBox(1.5, 1.5);
            //     //     fd.density = 100.0;
            //     //     fd.filter.categoryBits = 0x0002;
            //     //     bd.position.Set(1.0 * i, y);
            //     //     bd.angularDamping = 0.4;
            //     // }
            //     /*box2d.b2Body*/
            //     const body = this.m_world.CreateBody(bd);
            //     body.CreateFixture(fd);
            //     /*box2d.b2Vec2*/
            //     const anchor = new box2d.b2Vec2(i, y);
            //     jd.Initialize(prevBody, body, anchor);
            //     this.m_world.CreateJoint(jd);
            //     prevBody = body;
            // }
            // const anchor = ground.GetPosition();
            // // jd.Initialize(prevBody, sensor, anchor);
            // // this.m_world.CreateJoint(jd);
            // /*float32*/
            // const extraLength = 2.0;
            // this.m_ropeDef.maxLength = N - 1.0 + extraLength;
            // this.m_ropeDef.bodyB = sensor;
            // this.m_ropeDef.bodyB.GetLocalPoint(
            //     anchor,
            //     this.m_ropeDef.localAnchorB,
            // );
            // this.m_ropeDef.bodyA = ground;
        }

        // this.m_rope = this.m_world.CreateJoint(
        //     this.m_ropeDef,
        // ) as box2d.b2RopeJoint;
    }

    public BeginContact(contact: box2d.b2Contact) {
        const fixtureA = contact.GetFixtureA();
        const fixtureB = contact.GetFixtureB();

        let userData;

        const bodyA = fixtureA.GetBody();
        const bodyB = fixtureB.GetBody();

        if (fixtureA === this.sensor) {
            userData = bodyB.GetUserData();
        } else if (fixtureB === this.sensor) {
            userData = bodyA.GetUserData();
        } else if (fixtureA === this.teleport && this.bodies.includes(bodyB)) {
            this.toTeleport.add(bodyB);
        } else if (fixtureB === this.teleport && this.bodies.includes(bodyA)) {
            this.toTeleport.add(bodyA);
        }
        if (userData) {
            const touching = userData;
            touching[0] = true;
        }
    }

    public EndContact(contact: box2d.b2Contact) {
        const fixtureA = contact.GetFixtureA();
        const fixtureB = contact.GetFixtureB();

        let userData;
        if (fixtureA === this.sensor) {
            userData = fixtureB.GetBody().GetUserData();
        } else if (fixtureB === this.sensor) {
            userData = fixtureA.GetBody().GetUserData();
        }
        if (userData) {
            const touching = userData;
            touching[0] = false;
        }
    }

    public Step(settings: testbed.Settings): void {
        super.Step(settings);

        this.toTeleport.forEach((body) => {
            body.SetPositionXY(0, 30);
            body.SetLinearVelocity({ x: 0, y: 0 });
            this.toTeleport.delete(body);
        });
        const ground = this.sensor.GetBody();

        const circle = this.sensor.GetShape() as box2d.b2CircleShape;
        const center = ground.GetWorldPoint(circle.m_p, new box2d.b2Vec2());

        this.bodies.forEach((body) => {
            const data = body.GetUserData();
            if (!data || !data[0]) {
                return;
            }
            const position = body.GetPosition();

            const d = box2d.b2Vec2.SubVV(center, position, new box2d.b2Vec2());
            if (d.LengthSquared() < box2d.b2_epsilon_sq) {
                return;
            }

            d.Normalize();
            const F = box2d.b2Vec2.MulSV(800.0, d, new box2d.b2Vec2());
            body.ApplyForce(F, position);
        });
    }

    public static Create(): testbed.Test {
        return new Hoover();
    }
}
