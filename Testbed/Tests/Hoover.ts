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
    public static readonly e_count = 20;

    public m_sensor: box2d.b2Fixture;
    public teleport: box2d.b2Fixture;
    public m_bodies: box2d.b2Body[];
    public toTeleport: Set<box2d.b2Body> = new Set<box2d.b2Body>();

    public m_touching: boolean[][];

    constructor() {
        super();

        this.m_bodies = new Array(Hoover.e_count);
        this.m_touching = new Array(Hoover.e_count);
        for (let i = 0; i < Hoover.e_count; ++i) {
            this.m_touching[i] = new Array(1);
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
        {
            const shape = new box2d.b2CircleShape();
            shape.m_radius = 5.0;
            shape.m_p.Set(0.0, 10.0);

            const inner = new box2d.b2CircleShape();
            inner.m_radius = 1.0;
            inner.m_p.Set(0.0, 10.0);

            const bd = new box2d.b2BodyDef();
            bd.type = box2d.b2BodyType.b2_dynamicBody;
            const fd = new box2d.b2FixtureDef();
            fd.shape = shape;
            fd.isSensor = true;

            const fd2 = new box2d.b2FixtureDef();
            fd2.shape = inner;
            fd2.isSensor = true;
            const sensor = this.m_world.CreateBody(bd);
            sensor.SetGravityScale(0);
            this.m_sensor = sensor.CreateFixture(fd);
            this.teleport = sensor.CreateFixture(fd2);
        }

        {
            const shape = new box2d.b2CircleShape();
            shape.m_radius = 1.0;

            for (let i = 0; i < Hoover.e_count; ++i) {
                //const bd = new box2d.b2BodyDef();
                bd.type = box2d.b2BodyType.b2_dynamicBody;
                bd.position.Set(-10.0 + 3.0 * i, 20.0);
                bd.userData = this.m_touching[i];

                this.m_touching[i][0] = false;
                this.m_bodies[i] = this.m_world.CreateBody(bd);

                this.m_bodies[i].CreateFixture(shape, 1.0);
            }
        }
    }

    public BeginContact(contact: box2d.b2Contact) {
        const fixtureA = contact.GetFixtureA();
        const fixtureB = contact.GetFixtureB();

        let userData;

        const bodyA = fixtureA.GetBody();
        const bodyB = fixtureB.GetBody();

        if (fixtureA === this.m_sensor) {
            userData = bodyB.GetUserData();
        } else if (fixtureB === this.m_sensor) {
            userData = bodyA.GetUserData();
        } else if (
            fixtureA === this.teleport &&
            this.m_bodies.includes(bodyB)
        ) {
            this.toTeleport.add(bodyB);
        } else if (
            fixtureB === this.teleport &&
            this.m_bodies.includes(bodyA)
        ) {
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
        if (fixtureA === this.m_sensor) {
            userData = fixtureB.GetBody().GetUserData();
        } else if (fixtureB === this.m_sensor) {
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

        // Traverse the contact results. Apply a force on shapes
        // that overlap the sensor.
        for (let i = 0; i < Hoover.e_count; ++i) {
            if (!this.m_touching[i][0]) {
                continue;
            }

            const body = this.m_bodies[i];
            const ground = this.m_sensor.GetBody();

            const circle = this.m_sensor.GetShape() as box2d.b2CircleShape;
            const center = ground.GetWorldPoint(circle.m_p, new box2d.b2Vec2());

            const position = body.GetPosition();

            const d = box2d.b2Vec2.SubVV(center, position, new box2d.b2Vec2());
            if (d.LengthSquared() < box2d.b2_epsilon_sq) {
                continue;
            }

            d.Normalize();
            const F = box2d.b2Vec2.MulSV(800.0, d, new box2d.b2Vec2());
            body.ApplyForce(F, position);
        }
    }

    public static Create(): testbed.Test {
        return new Hoover();
    }
}
