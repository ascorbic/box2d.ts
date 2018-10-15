/*
 * Copyright (c) 2013 Google, Inc.
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

// #if B2_ENABLE_PARTICLE

import * as box2d from "Box2D";
import * as testbed from "Testbed";

export class Digging extends testbed.Test {
    public drawing: boolean = false;

    public destroyed = 0;
    constructor() {
        super();

        {
            const bd = new box2d.b2BodyDef();
            const ground = this.m_world.CreateBody(bd);

            const shape = new box2d.b2ChainShape();
            const vertices = [
                new box2d.b2Vec2(-2, 0),
                new box2d.b2Vec2(2, 0),
                new box2d.b2Vec2(2, 0.1),
                new box2d.b2Vec2(-2, 0.1),
            ];
            shape.CreateLoop(vertices, 4);
            ground.CreateFixture(shape, 0.0);
        }

        this.m_particleSystem.SetRadius(0.025 * 2); // HACK: increase particle radius
        this.m_particleSystem.SetDamping(0.2);

        // {
        //     const shape = new box2d.b2PolygonShape();
        //     shape.SetAsBox(0.8, 1.0, new box2d.b2Vec2(-1.2, 1.01), 0);
        //     const pd = new box2d.b2ParticleGroupDef();
        //     pd.flags = testbed.Test.GetParticleParameterValue();
        //     pd.shape = shape;
        //     const group = this.m_particleSystem.CreateParticleGroup(pd);
        //     if (pd.flags & box2d.b2ParticleFlag.b2_colorMixingParticle) {
        //         this.ColorParticleGroup(group, 0);
        //     }
        // }

        {
            const shape = new box2d.b2PolygonShape();
            shape.SetAsBox(2, 2);
            const pd = new box2d.b2ParticleGroupDef();
            // pd.flags = box2d.b2ParticleFlag.b2_elasticParticle;
            pd.groupFlags =
                box2d.b2ParticleGroupFlag.b2_rigidParticleGroup |
                box2d.b2ParticleGroupFlag.b2_solidParticleGroup;
            pd.position.Set(0, 2);
            pd.shape = shape;
            pd.color.Set(128, 0, 0, 1);
            this.m_particleSystem.CreateParticleGroup(pd);
        }

        {
            const shape = new box2d.b2CircleShape();
            shape.m_p.Set(0, 3);
            shape.m_radius = 0.5;
            const pd = new box2d.b2ParticleGroupDef();
            // pd.flags = box2d.b2ParticleFlag.b2_springParticle;
            // pd.groupFlags = box2d.b2ParticleGroupFlag.b2_solidParticleGroup;
            pd.shape = shape;
            pd.color.Set(0, 0, 1, 1);
            const xf = new box2d.b2Transform();
            xf.SetIdentity();
            this.m_particleSystem.DestroyParticlesInShape(shape, xf);

            this.m_particleSystem.CreateParticleGroup(pd);
        }
    }

    public spawnWater() {
        const shape = new box2d.b2CircleShape();
        shape.m_p.Set(0, 3.5);
        shape.m_radius = 0.1;
        const pd = new box2d.b2ParticleGroupDef();
        pd.shape = shape;
        pd.color.Set(0, 0, 1, 1);
        const xf = new box2d.b2Transform();
        xf.SetIdentity();
        this.m_particleSystem.DestroyParticlesInShape(shape, xf);

        const pg = this.m_particleSystem.CreateParticleGroup(pd);
        this.destroyed -= pg.GetParticleCount();
    }

    public GetDefaultViewZoom() {
        return 0.1;
    }

    public MouseDown() {
        this.drawing = true;
    }

    public MouseUp() {
        this.drawing = false;
    }
    public MouseMove(p: box2d.b2Vec2) {
        super.MouseMove(p);
        if (this.drawing) {
            const shape = new box2d.b2CircleShape();
            shape.m_p.Copy(p);
            shape.m_radius = 0.05;
            ///  b2Transform xf;
            ///  xf.SetIdentity();
            const xf = box2d.b2Transform.IDENTITY;

            this.destroyed += this.m_particleSystem.DestroyParticlesInShape(
                shape,
                xf,
            );
        }
    }

    public static Create() {
        return new Digging();
    }

    public Step(settings: testbed.Settings): void {
        super.Step(settings);
        if (this.destroyed > 0) {
            this.spawnWater();
        }
    }
}

// /**
//  * Keep track of particle groups in a set, removing them when
//  * they're destroyed.
//  */
// export class ParticleGroupTracker extends box2d.b2DestructionListener {
//     public m_particleGroups: box2d.b2ParticleGroup[] = [];

//     /**
//      * Called when any particle group is about to be destroyed.
//      */
//     public SayGoodbyeParticleGroup(group: box2d.b2ParticleGroup): void {
//         this.RemoveParticleGroup(group);
//     }

//     /**
//      * Add a particle group to the tracker.
//      */
//     public AddParticleGroup(group: box2d.b2ParticleGroup): void {
//         this.m_particleGroups.push(group);
//     }

//     /**
//      * Remove a particle group from the tracker.
//      */
//     public RemoveParticleGroup(group: box2d.b2ParticleGroup): void {
//         this.m_particleGroups.splice(this.m_particleGroups.indexOf(group), 1);
//     }

//     public GetParticleGroups(): box2d.b2ParticleGroup[] {
//         return this.m_particleGroups;
//     }
// }

// export class DestructionListener extends ParticleGroupTracker {
//     public world: box2d.b2World;
//     public previousListener: box2d.b2DestructionListener | null = null;

//     /**
//      * Initialize the score.
//      */
//     public __ctor__() {}

//     /**
//      * Initialize the particle system and world, setting this class
//      * as a destruction listener for the world.
//      */
//     constructor(world: box2d.b2World) {
//         super();
//         // DEBUG: box2d.b2Assert(world !== null);
//         this.world = world;
//         this.previousListener = world.m_destructionListener;
//         this.world.SetDestructionListener(this);
//     }

//     public __dtor__() {
//         if (this.world) {
//             this.world.SetDestructionListener(this.previousListener);
//         }
//     }

//     /**
//      * Update the score when certain particles are destroyed.
//      */
//     public SayGoodbyeParticle(
//         particleSystem: box2d.b2ParticleSystem,
//         index: number,
//     ): void {}
// }

// #endif
