export enum b2ShapeType {
    "b2Shape::e_circle",
    "b2Shape::e_edge",
    "b2Shape::e_polygon",
    "b2Shape::e_chain",
    "b2Shape::e_typeCount"
}
export enum b2BodyType {
    "b2_staticBody",
    "b2_kinematicBody",
    "b2_dynamicBody"
}
export enum b2JointType {
    "e_unknownJoint",
    "e_revoluteJoint",
    "e_prismaticJoint",
    "e_distanceJoint",
    "e_pulleyJoint",
    "e_mouseJoint",
    "e_gearJoint",
    "e_wheelJoint",
    "e_weldJoint",
    "e_frictionJoint",
    "e_ropeJoint",
    "e_motorJoint"
}
export enum b2LimitState {
    "e_inactiveLimit",
    "e_atLowerLimit",
    "e_atUpperLimit",
    "e_equalLimits"
}
export enum b2ContactFeatureType {
    "b2ContactFeature::e_vertex",
    "b2ContactFeature::e_face"
}
export enum b2DrawFlag {
    "b2Draw::e_shapeBit",
    "b2Draw::e_jointBit",
    "b2Draw::e_aabbBit",
    "b2Draw::e_pairBit",
    "b2Draw::e_centerOfMassBit"
}
export enum b2ManifoldType {
    "b2Manifold::e_circles",
    "b2Manifold::e_faceA",
    "b2Manifold::e_faceB"
}
export class b2Contact {
    GetManifold(): b2Manifold;
    IsTouching(): boolean;
    SetEnabled(flag: boolean): void;
    IsEnabled(): boolean;
    GetNext(): b2Contact;
    GetFixtureA(): b2Fixture;
    GetChildIndexA(): number;
    GetFixtureB(): b2Fixture;
    GetChildIndexB(): number;
    SetFriction(friction: number): void;
    GetFriction(): number;
    ResetFriction(): void;
    SetRestitution(restitution: number): void;
    GetRestitution(): number;
    ResetRestitution(): void;
    SetTangentSpeed(speed: number): void;
    GetTangentSpeed(): number;
}

export class b2ContactListener {
    $__dummyprop__b2ContactListener: any;
}

export class JSContactListener {
    JSContactListener(): void;
    BeginContact(contact: b2Contact): void;
    EndContact(contact: b2Contact): void;
    BeginParticleBodyContact(
        particleSystem: b2ParticleSystem,
        particleBodyContact: b2ParticleBodyContact
    ): void;
    EndParticleBodyContact(
        fixture: b2Fixture,
        particleSystem: b2ParticleSystem,
        index: number
    ): void;
    BeginParticleContact(
        particleSystem: b2ParticleSystem,
        particleContact: b2ParticleContact
    ): void;
    EndParticleContact(
        particleSystem: b2ParticleSystem,
        indexA: number,
        indexB: number
    ): void;
}

export class b2World {
    b2World(gravity: b2Vec2): void;
    SetDestructionListener(listener: b2DestructionListener): void;
    SetContactFilter(filter: JSContactFilter): void;
    SetContactListener(listener: JSContactListener): void;
    SetDebugDraw(debugDraw: b2Draw): void;
    CreateBody(def: b2BodyDef): b2Body;
    DestroyBody(body: b2Body): void;
    CreateJoint(def: b2JointDef): b2Joint;
    DestroyJoint(joint: b2Joint): void;
    CreateParticleSystem(def: b2ParticleSystemDef): b2ParticleSystem;
    DestroyParticleSystem(system: b2ParticleSystem): void;
    Step(
        timeStep: number,
        velocityIterations: number,
        positionIterations: number,
        particleIterations: number
    ): void;
    Step(
        timeStep: number,
        velocityIterations: number,
        positionIterations: number
    ): void;
    CalculateReasonableParticleIterations(timeStep: number): number;
    ClearForces(): void;
    DrawDebugData(): void;
    QueryAABB(callback: b2QueryCallback, aabb: b2AABB): void;
    RayCast(callback: b2RayCastCallback, point1: b2Vec2, point2: b2Vec2): void;
    GetBodyList(): b2Body;
    GetJointList(): b2Joint;
    GetParticleSystemList(): b2ParticleSystem;
    GetContactList(): b2Contact;
    SetAllowSleeping(flag: boolean): void;
    GetAllowSleeping(): boolean;
    SetWarmStarting(flag: boolean): void;
    GetWarmStarting(): boolean;
    SetContinuousPhysics(flag: boolean): void;
    GetContinuousPhysics(): boolean;
    SetSubStepping(flag: boolean): void;
    GetSubStepping(): boolean;
    GetProxyCount(): number;
    GetBodyCount(): number;
    GetJointCount(): number;
    GetContactCount(): number;
    GetTreeHeight(): number;
    GetTreeBalance(): number;
    GetTreeQuality(): number;
    SetGravity(gravity: b2Vec2): void;
    GetGravity(): b2Vec2;
    IsLocked(): boolean;
    SetAutoClearForces(flag: boolean): void;
    GetAutoClearForces(): boolean;
    GetProfile(): b2Profile;
    Dump(): void;
}

export class b2Shape {
    m_type: b2ShapeType;
    m_radius: number;
    GetType(): b2ShapeType;
    GetChildCount(): number;
    TestPoint(xf: b2Transform, p: b2Vec2): boolean;
    RayCast(
        output: b2RayCastOutput,
        input: b2RayCastInput,
        transform: b2Transform,
        childIndex: number
    ): boolean;
    ComputeAABB(aabb: b2AABB, xf: b2Transform, childIndex: number): void;
    ComputeMass(massData: b2MassData, density: number): void;
}

export class b2FixtureDef {
    shape: b2Shape;
    userData: any;
    friction: number;
    restitution: number;
    density: number;
    isSensor: boolean;
    filter: b2Filter;
    b2FixtureDef(): void;
}

export class b2Fixture {
    GetType(): b2ShapeType;
    GetShape(): b2Shape;
    SetSensor(sensor: boolean): void;
    IsSensor(): boolean;
    SetFilterData(filter: b2Filter): void;
    GetFilterData(): b2Filter;
    Refilter(): void;
    GetBody(): b2Body;
    GetNext(): b2Fixture;
    GetUserData(): any;
    SetUserData(data: any): void;
    TestPoint(p: b2Vec2): boolean;
    RayCast(
        output: b2RayCastOutput,
        input: b2RayCastInput,
        childIndex: number
    ): boolean;
    GetMassData(massData: b2MassData): void;
    SetDensity(density: number): void;
    GetDensity(): number;
    GetFriction(): number;
    SetFriction(friction: number): void;
    GetRestitution(): number;
    SetRestitution(restitution: number): void;
    GetAABB(childIndex: number): b2AABB;
    Dump(bodyIndex: number): void;
}

export class b2Transform {
    p: b2Vec2;
    q: b2Rot;
    b2Transform(): void;
    b2Transform(position: b2Vec2, rotation: b2Rot): void;
    SetIdentity(): void;
    Set(position: b2Vec2, angle: number): void;
}

export class b2RayCastCallback {
    $__dummyprop__b2RayCastCallback: any;
}

export class JSRayCastCallback {
    JSRayCastCallback(): void;
    ReportFixture(
        fixture: b2Fixture,
        point: b2Vec2,
        normal: b2Vec2,
        fraction: number
    ): number;
}

export class b2QueryCallback {
    $__dummyprop__b2QueryCallback: any;
}

export class JSQueryCallback {
    JSQueryCallback(): void;
    ReportFixture(fixture: b2Fixture): boolean;
}

export class b2MassData {
    mass: number;
    center: b2Vec2;
    I: number;
    b2MassData(): void;
}

export class b2Vec2 {
    x: number;
    y: number;
    b2Vec2(x: number, y: number): void;
    SetZero(): void;
    Set(x: number, y: number): void;
    op_add(v: b2Vec2): void;
    op_sub(v: b2Vec2): void;
    op_mul(s: number): void;
    Length(): number;
    LengthSquared(): number;
    Normalize(): number;
    IsValid(): boolean;
    Skew(): b2Vec2;
}

export class b2Vec3 {
    x: number;
    y: number;
    z: number;
    b2Vec3(): void;
    b2Vec3(x: number, y: number, z: number): void;
    SetZero(): void;
    Set(x: number, y: number, z: number): void;
    op_add(v: b2Vec3): void;
    op_sub(v: b2Vec3): void;
    op_mul(s: number): void;
}

export class b2Body {
    CreateFixture(def: b2FixtureDef): b2Fixture;
    CreateFixture(shape: b2Shape, density: number): b2Fixture;
    DestroyFixture(fixture: b2Fixture): void;
    SetTransform(position: b2Vec2, angle: number): void;
    GetTransform(): b2Transform;
    GetPosition(): b2Vec2;
    GetAngle(): number;
    GetWorldCenter(): b2Vec2;
    GetLocalCenter(): b2Vec2;
    SetLinearVelocity(v: b2Vec2): void;
    GetLinearVelocity(): b2Vec2;
    SetAngularVelocity(omega: number): void;
    GetAngularVelocity(): number;
    ApplyForce(force: b2Vec2, point: b2Vec2, awake: boolean): void;
    ApplyForceToCenter(force: b2Vec2, awake: boolean): void;
    ApplyTorque(torque: number, awake: boolean): void;
    ApplyLinearImpulse(impulse: b2Vec2, point: b2Vec2, awake: boolean): void;
    ApplyAngularImpulse(impulse: number, awake: boolean): void;
    GetMass(): number;
    GetInertia(): number;
    GetMassData(data: b2MassData): void;
    SetMassData(data: b2MassData): void;
    ResetMassData(): void;
    GetWorldPoint(localPoint: b2Vec2): b2Vec2;
    GetWorldVector(localVector: b2Vec2): b2Vec2;
    GetLocalPoint(worldPoint: b2Vec2): b2Vec2;
    GetLocalVector(worldVector: b2Vec2): b2Vec2;
    GetLinearVelocityFromWorldPoint(worldPoint: b2Vec2): b2Vec2;
    GetLinearVelocityFromLocalPoint(localPoint: b2Vec2): b2Vec2;
    GetLinearDamping(): number;
    SetLinearDamping(linearDamping: number): void;
    GetAngularDamping(): number;
    SetAngularDamping(angularDamping: number): void;
    GetGravityScale(): number;
    SetGravityScale(scale: number): void;
    SetType(type: b2BodyType): void;
    GetType(): b2BodyType;
    SetBullet(flag: boolean): void;
    IsBullet(): boolean;
    SetSleepingAllowed(flag: boolean): void;
    IsSleepingAllowed(): boolean;
    SetAwake(flag: boolean): void;
    IsAwake(): boolean;
    SetActive(flag: boolean): void;
    IsActive(): boolean;
    SetFixedRotation(flag: boolean): void;
    IsFixedRotation(): boolean;
    GetFixtureList(): b2Fixture;
    GetJointList(): b2JointEdge;
    GetContactList(): b2ContactEdge;
    GetNext(): b2Body;
    GetUserData(): any;
    SetUserData(data: any): void;
    GetWorld(): b2World;
    Dump(): void;
}

export class b2BodyDef {
    type: b2BodyType;
    position: b2Vec2;
    angle: number;
    linearVelocity: b2Vec2;
    angularVelocity: number;
    linearDamping: number;
    angularDamping: number;
    allowSleep: boolean;
    awake: boolean;
    fixedRotation: boolean;
    bullet: boolean;
    active: boolean;
    userData: any;
    gravityScale: number;
    b2BodyDef(): void;
}

export class b2Filter {
    categoryBits: number;
    maskBits: number;
    groupIndex: number;
    b2Filter(): void;
}

export class b2AABB {
    lowerBound: b2Vec2;
    upperBound: b2Vec2;
    b2AABB(): void;
    IsValid(): boolean;
    GetCenter(): b2Vec2;
    GetExtents(): b2Vec2;
    GetPerimeter(): number;
    Combine(aabb: b2AABB): void;
    Combine(aabb1: b2AABB, aabb2: b2AABB): void;
    Contains(aabb: b2AABB): boolean;
    RayCast(output: b2RayCastOutput, input: b2RayCastInput): boolean;
}

export class b2CircleShape extends b2Shape {
    m_p: b2Vec2;
    b2CircleShape(): void;
}

export class b2EdgeShape extends b2Shape {
    m_vertex1: b2Vec2;
    m_vertex2: b2Vec2;
    m_vertex0: b2Vec2;
    m_vertex3: b2Vec2;
    m_hasVertex0: boolean;
    m_hasVertex3: boolean;
    b2EdgeShape(): void;
    Set(v1: b2Vec2, v2: b2Vec2): void;
}

export class b2JointDef {
    type: b2JointType;
    userData: any;
    bodyA: b2Body;
    bodyB: b2Body;
    collideConnected: boolean;
    b2JointDef(): void;
}

export class b2Joint {
    GetType(): b2JointType;
    GetBodyA(): b2Body;
    GetBodyB(): b2Body;
    GetAnchorA(): b2Vec2;
    GetAnchorB(): b2Vec2;
    GetReactionForce(inv_dt: number): b2Vec2;
    GetReactionTorque(inv_dt: number): number;
    GetNext(): b2Joint;
    GetUserData(): any;
    SetUserData(data: any): void;
    IsActive(): boolean;
    GetCollideConnected(): boolean;
    Dump(): void;
}

export class b2WeldJoint extends b2Joint {
    GetLocalAnchorA(): b2Vec2;
    GetLocalAnchorB(): b2Vec2;
    SetFrequency(hz: number): void;
    GetFrequency(): number;
    SetDampingRatio(ratio: number): void;
    GetDampingRatio(): number;
    Dump(): void;
}

export class b2WeldJointDef extends b2JointDef {
    localAnchorA: b2Vec2;
    localAnchorB: b2Vec2;
    referenceAngle: number;
    frequencyHz: number;
    dampingRatio: number;
    b2WeldJointDef(): void;
    Initialize(bodyA: b2Body, bodyB: b2Body, anchor: b2Vec2): void;
}

export class b2ChainShape extends b2Shape {
    m_vertices: b2Vec2;
    m_count: number;
    m_prevVertex: b2Vec2;
    m_nextVertex: b2Vec2;
    m_hasPrevVertex: boolean;
    m_hasNextVertex: boolean;
    b2ChainShape(): void;
    CreateLoop(vertices: b2Vec2, count: number): void;
    CreateChain(vertices: b2Vec2, count: number): void;
    SetPrevVertex(prevVertex: b2Vec2): void;
    SetNextVertex(nextVertex: b2Vec2): void;
    GetChildEdge(edge: b2EdgeShape, index: number): void;
}

export class b2Color {
    r: number;
    g: number;
    b: number;
    b2Color(): void;
    b2Color(r: number, g: number, b: number): void;
    Set(ri: number, gi: number, bi: number): void;
}

export class b2ContactEdge {
    other: b2Body;
    contact: b2Contact;
    prev: b2ContactEdge;
    next: b2ContactEdge;
    b2ContactEdge(): void;
}

export class b2ContactFeature {
    indexA: number;
    indexB: number;
    typeA: number;
    typeB: number;
}

export class b2ContactFilter {
    $__dummyprop__b2ContactFilter: any;
}

export class JSContactFilter {
    JSContactFilter(): void;
    ShouldCollide(fixtureA: b2Fixture, fixtureB: b2Fixture): boolean;
}

export class b2ContactID {
    cf: b2ContactFeature;
    key: number;
}

export class b2ContactImpulse {
    count: number;
}

export class b2DestructionListener {
    $__dummyprop__b2DestructionListener: any;
}

export class b2DestructionListenerWrapper {
    $__dummyprop__b2DestructionListenerWrapper: any;
}

export class JSDestructionListener {
    JSDestructionListener(): void;
    SayGoodbyeJoint(joint: b2Joint): void;
    SayGoodbyeFixture(joint: b2Fixture): void;
}

export class b2DistanceJoint extends b2Joint {
    GetLocalAnchorA(): b2Vec2;
    GetLocalAnchorB(): b2Vec2;
    SetLength(length: number): void;
    GetLength(): number;
    SetFrequency(hz: number): void;
    GetFrequency(): number;
    SetDampingRatio(ratio: number): void;
    GetDampingRatio(): number;
}

export class b2DistanceJointDef extends b2JointDef {
    localAnchorA: b2Vec2;
    localAnchorB: b2Vec2;
    length: number;
    frequencyHz: number;
    dampingRatio: number;
    b2DistanceJointDef(): void;
    Initialize(
        bodyA: b2Body,
        bodyB: b2Body,
        anchorA: b2Vec2,
        anchorB: b2Vec2
    ): void;
}

export class b2Draw {
    SetFlags(flags: number): void;
    GetFlags(): number;
    AppendFlags(flags: number): void;
    ClearFlags(flags: number): void;
}

export class JSDraw {
    JSDraw(): void;
    DrawPolygon(vertices: b2Vec2, vertexCount: number, color: b2Color): void;
    DrawSolidPolygon(
        vertices: b2Vec2,
        vertexCount: number,
        color: b2Color
    ): void;
    DrawCircle(center: b2Vec2, radius: number, color: b2Color): void;
    DrawSolidCircle(
        center: b2Vec2,
        radius: number,
        axis: b2Vec2,
        color: b2Color
    ): void;
    DrawSegment(p1: b2Vec2, p2: b2Vec2, color: b2Color): void;
    DrawTransform(xf: b2Transform): void;
    DrawParticles(
        centers: b2Vec2,
        radius: number,
        colors: b2ParticleColor,
        count: number
    ): void;
}

export class b2FrictionJoint extends b2Joint {
    GetLocalAnchorA(): b2Vec2;
    GetLocalAnchorB(): b2Vec2;
    SetMaxForce(force: number): void;
    GetMaxForce(): number;
    SetMaxTorque(torque: number): void;
    GetMaxTorque(): number;
}

export class b2FrictionJointDef extends b2JointDef {
    localAnchorA: b2Vec2;
    localAnchorB: b2Vec2;
    maxForce: number;
    maxTorque: number;
    b2FrictionJointDef(): void;
    Initialize(bodyA: b2Body, bodyB: b2Body, anchor: b2Vec2): void;
}

export class b2GearJoint extends b2Joint {
    GetJoint1(): b2Joint;
    GetJoint2(): b2Joint;
    SetRatio(ratio: number): void;
    GetRatio(): number;
}

export class b2GearJointDef extends b2JointDef {
    joint1: b2Joint;
    joint2: b2Joint;
    ratio: number;
    b2GearJointDef(): void;
}

export class b2JointEdge {
    other: b2Body;
    joint: b2Joint;
    prev: b2JointEdge;
    next: b2JointEdge;
    b2JointEdge(): void;
}

export class b2Manifold {
    localNormal: b2Vec2;
    localPoint: b2Vec2;
    type: b2ManifoldType;
    pointCount: number;
    b2Manifold(): void;
}

export class b2ManifoldPoint {
    localPoint: b2Vec2;
    normalImpulse: number;
    tangentImpulse: number;
    id: b2ContactID;
    b2ManifoldPoint(): void;
}

export class b2Mat22 {
    ex: b2Vec2;
    ey: b2Vec2;
    b2Mat22(): void;
    b2Mat22(c1: b2Vec2, c2: b2Vec2): void;
    b2Mat22(a11: number, a12: number, a21: number, a22: number): void;
    Set(c1: b2Vec2, c2: b2Vec2): void;
    SetIdentity(): void;
    SetZero(): void;
    GetInverse(): b2Mat22;
    Solve(b: b2Vec2): b2Vec2;
}

export class b2Mat33 {
    ex: b2Vec3;
    ey: b2Vec3;
    ez: b2Vec3;
    b2Mat33(): void;
    b2Mat33(c1: b2Vec3, c2: b2Vec3, c3: b2Vec3): void;
    SetZero(): void;
    Solve33(b: b2Vec3): b2Vec3;
    Solve22(b: b2Vec2): b2Vec2;
    GetInverse22(M: b2Mat33): void;
    GetSymInverse33(M: b2Mat33): void;
}

export class b2MouseJoint extends b2Joint {
    SetTarget(target: b2Vec2): void;
    GetTarget(): b2Vec2;
    SetMaxForce(force: number): void;
    GetMaxForce(): number;
    SetFrequency(hz: number): void;
    GetFrequency(): number;
    SetDampingRatio(ratio: number): void;
    GetDampingRatio(): number;
}

export class b2MouseJointDef extends b2JointDef {
    target: b2Vec2;
    maxForce: number;
    frequencyHz: number;
    dampingRatio: number;
    b2MouseJointDef(): void;
}

export class b2PolygonShape {
    m_centroid: b2Vec2;
    m_count: number;
    b2PolygonShape(): void;
    Set(vertices: b2Vec2, vertexCount: number): void;
    SetAsBox(hx: number, hy: number): void;
    SetAsBox(hx: number, hy: number, center: b2Vec2, angle: number): void;
    GetVertexCount(): number;
    GetVertex(index: number): b2Vec2;
}

export class b2PrismaticJoint extends b2Joint {
    GetLocalAnchorA(): b2Vec2;
    GetLocalAnchorB(): b2Vec2;
    GetLocalAxisA(): b2Vec2;
    GetReferenceAngle(): number;
    GetJointTranslation(): number;
    GetJointSpeed(): number;
    IsLimitEnabled(): boolean;
    EnableLimit(flag: boolean): void;
    GetLowerLimit(): number;
    GetUpperLimit(): number;
    SetLimits(lower: number, upper: number): void;
    IsMotorEnabled(): boolean;
    EnableMotor(flag: boolean): void;
    SetMotorSpeed(speed: number): void;
    GetMotorSpeed(): number;
    SetMaxMotorForce(force: number): void;
    GetMaxMotorForce(): number;
    GetMotorForce(inv_dt: number): number;
}

export class b2PrismaticJointDef extends b2JointDef {
    localAnchorA: b2Vec2;
    localAnchorB: b2Vec2;
    localAxisA: b2Vec2;
    referenceAngle: number;
    enableLimit: boolean;
    lowerTranslation: number;
    upperTranslation: number;
    enableMotor: boolean;
    maxMotorForce: number;
    motorSpeed: number;
    b2PrismaticJointDef(): void;
    Initialize(
        bodyA: b2Body,
        bodyB: b2Body,
        anchor: b2Vec2,
        axis: b2Vec2
    ): void;
}

export class b2Profile {
    step: number;
    collide: number;
    solve: number;
    solveInit: number;
    solveVelocity: number;
    solvePosition: number;
    broadphase: number;
    solveTOI: number;
}

export class b2PulleyJoint extends b2Joint {
    GetGroundAnchorA(): b2Vec2;
    GetGroundAnchorB(): b2Vec2;
    GetLengthA(): number;
    GetLengthB(): number;
    GetRatio(): number;
    GetCurrentLengthA(): number;
    GetCurrentLengthB(): number;
}

export class b2PulleyJointDef extends b2JointDef {
    groundAnchorA: b2Vec2;
    groundAnchorB: b2Vec2;
    localAnchorA: b2Vec2;
    localAnchorB: b2Vec2;
    lengthA: number;
    lengthB: number;
    ratio: number;
    b2PulleyJointDef(): void;
    Initialize(
        bodyA: b2Body,
        bodyB: b2Body,
        groundAnchorA: b2Vec2,
        groundAnchorB: b2Vec2,
        anchorA: b2Vec2,
        anchorB: b2Vec2,
        ratio: number
    ): void;
}

export class b2RayCastInput {
    p1: b2Vec2;
    p2: b2Vec2;
    maxFraction: number;
}

export class b2RayCastOutput {
    normal: b2Vec2;
    fraction: number;
}

export class b2RevoluteJoint extends b2Joint {
    GetLocalAnchorA(): b2Vec2;
    GetLocalAnchorB(): b2Vec2;
    GetReferenceAngle(): number;
    GetJointAngle(): number;
    GetJointSpeed(): number;
    IsLimitEnabled(): boolean;
    EnableLimit(flag: boolean): void;
    GetLowerLimit(): number;
    GetUpperLimit(): number;
    SetLimits(lower: number, upper: number): void;
    IsMotorEnabled(): boolean;
    EnableMotor(flag: boolean): void;
    SetMotorSpeed(speed: number): void;
    GetMotorSpeed(): number;
    SetMaxMotorTorque(torque: number): void;
    GetMaxMotorTorque(): number;
    GetMotorTorque(inv_dt: number): number;
}

export class b2RevoluteJointDef extends b2JointDef {
    localAnchorA: b2Vec2;
    localAnchorB: b2Vec2;
    referenceAngle: number;
    enableLimit: boolean;
    lowerAngle: number;
    upperAngle: number;
    enableMotor: boolean;
    motorSpeed: number;
    maxMotorTorque: number;
    b2RevoluteJointDef(): void;
    Initialize(bodyA: b2Body, bodyB: b2Body, anchor: b2Vec2): void;
}

export class b2RopeJoint extends b2Joint {
    GetLocalAnchorA(): b2Vec2;
    GetLocalAnchorB(): b2Vec2;
    SetMaxLength(length: number): void;
    GetMaxLength(): number;
    GetLimitState(): b2LimitState;
}

export class b2RopeJointDef extends b2JointDef {
    localAnchorA: b2Vec2;
    localAnchorB: b2Vec2;
    maxLength: number;
    b2RopeJointDef(): void;
}

export class b2Rot {
    s: number;
    c: number;
    b2Rot(): void;
    b2Rot(angle: number): void;
    Set(angle: number): void;
    SetIdentity(): void;
    GetAngle(): number;
    GetXAxis(): b2Vec2;
    GetYAxis(): b2Vec2;
}

export class b2WheelJoint extends b2JointDef {
    GetLocalAnchorA(): b2Vec2;
    GetLocalAnchorB(): b2Vec2;
    GetLocalAxisA(): b2Vec2;
    GetJointTranslation(): number;
    GetJointSpeed(): number;
    IsMotorEnabled(): boolean;
    EnableMotor(flag: boolean): void;
    SetMotorSpeed(speed: number): void;
    GetMotorSpeed(): number;
    SetMaxMotorTorque(torque: number): void;
    GetMaxMotorTorque(): number;
    GetMotorTorque(inv_dt: number): number;
    SetSpringFrequencyHz(hz: number): void;
    GetSpringFrequencyHz(): number;
    SetSpringDampingRatio(ratio: number): void;
    GetSpringDampingRatio(): number;
}

export class b2WheelJointDef extends b2JointDef {
    localAnchorA: b2Vec2;
    localAnchorB: b2Vec2;
    localAxisA: b2Vec2;
    enableMotor: boolean;
    maxMotorTorque: number;
    motorSpeed: number;
    frequencyHz: number;
    dampingRatio: number;
    b2WheelJointDef(): void;
    Initialize(
        bodyA: b2Body,
        bodyB: b2Body,
        anchor: b2Vec2,
        axis: b2Vec2
    ): void;
}

export class b2MotorJoint extends b2Joint {
    SetLinearOffset(linearOffset: b2Vec2): void;
    GetLinearOffset(): b2Vec2;
    SetAngularOffset(angularOffset: number): void;
    GetAngularOffset(): number;
    SetMaxForce(force: number): void;
    GetMaxForce(): number;
    SetMaxTorque(torque: number): void;
    GetMaxTorque(): number;
    SetCorrectionFactor(factor: number): void;
    GetCorrectionFactor(): number;
}

export class b2MotorJointDef extends b2JointDef {
    linearOffset: b2Vec2;
    angularOffset: number;
    maxForce: number;
    maxTorque: number;
    correctionFactor: number;
    b2MotorJointDef(): void;
    Initialize(bodyA: b2Body, bodyB: b2Body): void;
}

export class b2ParticleColor {
    r: number;
    g: number;
    b: number;
    a: number;
    b2ParticleColor(): void;
    b2ParticleColor(r: number, g: number, b: number, a: number): void;
    Set(ri: number, gi: number, bi: number, ai: number): void;
}

export class b2ParticleDef {
    flags: number;
    position: b2Vec2;
    velocity: b2Vec2;
    color: b2ParticleColor;
    lifetime: number;
    userData: any;
    group: b2ParticleGroup;
    b2ParticleDef(): void;
}

export class b2ParticleGroupDef {
    flags: number;
    groupFlags: number;
    position: b2Vec2;
    angle: number;
    linearVelocity: b2Vec2;
    angularVelocity: number;
    color: b2ParticleColor;
    strength: number;
    shape: b2Shape;
    shapeCount: number;
    stride: number;
    particleCount: number;
    positionData: b2Vec2;
    lifetime: number;
    userData: any;
    group: b2ParticleGroup;
    b2ParticleGroupDef(): void;
}

export class b2ParticleGroup {
    GetNext(): b2ParticleGroup;
    GetParticleSystem(): b2ParticleSystem;
    GetParticleCount(): number;
    GetBufferIndex(): number;
    ContainsParticle(index: number): boolean;
    GetAllParticleFlags(): number;
    GetGroupFlags(): number;
    SetGroupFlags(flags: number): void;
    GetMass(): number;
    GetInertia(): number;
    GetCenter(): b2Vec2;
    GetLinearVelocity(): b2Vec2;
    GetAngularVelocity(): number;
    GetTransform(): b2Transform;
    GetPosition(): b2Vec2;
    GetAngle(): number;
    GetLinearVelocityFromWorldPoint(worldPoint: b2Vec2): b2Vec2;
    GetUserData(): any;
    SetUserData(data: any): void;
    ApplyForce(force: b2Vec2): void;
    ApplyLinearImpulse(impulse: b2Vec2): void;
    DestroyParticles(): void;
}

export class b2ParticleSystemDef {
    strictContactCheck: boolean;
    density: number;
    gravityScale: number;
    radius: number;
    maxCount: number;
    pressureStrength: number;
    dampingStrength: number;
    elasticStrength: number;
    springStrength: number;
    viscousStrength: number;
    surfaceTensionPressureStrength: number;
    surfaceTensionNormalStrength: number;
    repulsiveStrength: number;
    powderStrength: number;
    ejectionStrength: number;
    staticPressureStrength: number;
    staticPressureRelaxation: number;
    staticPressureIterations: number;
    colorMixingStrength: number;
    destroyByAge: boolean;
    lifetimeGranularity: number;
    b2ParticleSystemDef(): void;
}

export class b2ParticleSystem {
    CreateParticle(def: b2ParticleDef): number;
    DestroyParticle(index: number): void;
    DestroyOldestParticle(
        index: number,
        callDestructionListener: boolean
    ): void;
    DestroyParticlesInShape(shape: b2Shape, xf: b2Transform): number;
    CreateParticleGroup(def: b2ParticleGroupDef): b2ParticleGroup;
    JoinParticleGroups(groupA: b2ParticleGroup, groupB: b2ParticleGroup): void;
    SplitParticleGroup(group: b2ParticleGroup): void;
    GetParticleGroupList(): b2ParticleGroup;
    GetParticleGroupCount(): number;
    GetParticleCount(): number;
    GetMaxParticleCount(): number;
    SetMaxParticleCount(count: number): void;
    GetAllParticleFlags(): number;
    GetAllGroupFlags(): number;
    SetPaused(paused: boolean): void;
    GetPaused(): boolean;
    SetDensity(density: number): void;
    GetDensity(): number;
    SetGravityScale(gravityScale: number): void;
    GetGravityScale(): number;
    SetDamping(damping: number): void;
    GetDamping(): number;
    SetStaticPressureIterations(iterations: number): void;
    GetStaticPressureIterations(): number;
    SetRadius(radius: number): void;
    GetRadius(): number;
    GetPositionBuffer(): b2Vec2[];
    GetVelocityBuffer(): b2Vec2[];
    GetColorBuffer(): b2ParticleColor[];
    GetUserDataBuffer(): any[];
    SetParticleFlags(index: number, flags: number): void;
    GetParticleFlags(index: number): number;
    GetContactCount(): number;
    GetBodyContacts(): b2ParticleBodyContact;
}

export class b2ParticleContact {
    $__dummyprop__b2ParticleContact: any;
}

export class b2ParticleBodyContact {
    index: number;
    body: b2Body;
    fixture: b2Fixture;
    weight: number;
    normal: b2Vec2;
    mass: number;
    b2ParticleBodyContact(): void;
}

export class b2ParticlePair {
    $__dummyprop__b2ParticlePair: any;
}

export class b2ParticleTriad {
    $__dummyprop__b2ParticleTriad: any;
}

export class Box2D {
    b2ShapeType: typeof b2ShapeType;
    b2BodyType: typeof b2BodyType;
    b2JointType: typeof b2JointType;
    b2LimitState: typeof b2LimitState;
    b2ContactFeatureType: b2ContactFeatureType;
    b2DrawFlag: typeof b2DrawFlag;
    b2ManifoldType: b2ManifoldType;
    b2Contact: typeof b2Contact;
    b2ContactListener: typeof b2ContactListener;
    JSContactListener: typeof JSContactListener;
    b2World: typeof b2World;
    b2Shape: typeof b2Shape;
    b2FixtureDef: typeof b2FixtureDef;
    b2Fixture: typeof b2Fixture;
    b2Transform: typeof b2Transform;
    b2RayCastCallback: typeof b2RayCastCallback;
    JSRayCastCallback: typeof JSRayCastCallback;
    b2QueryCallback: typeof b2QueryCallback;
    JSQueryCallback: JSQueryCallback;
    b2MassData: typeof b2MassData;
    b2Vec2: typeof b2Vec2;
    b2Vec3: typeof b2Vec3;
    b2Body: typeof b2Body;
    b2BodyDef: typeof b2BodyDef;
    b2Filter: typeof b2Filter;
    b2AABB: typeof b2AABB;
    b2CircleShape: typeof b2CircleShape;
    b2EdgeShape: typeof b2EdgeShape;
    b2JointDef: typeof b2JointDef;
    b2Joint: typeof b2Joint;
    b2WeldJoint: typeof b2WeldJoint;
    b2WeldJointDef: typeof b2WeldJointDef;
    b2ChainShape: typeof b2ChainShape;
    b2Color: typeof b2Color;
    b2ContactEdge: typeof b2ContactEdge;
    b2ContactFeature: typeof b2ContactFeature;
    b2ContactFilter: typeof b2ContactFilter;
    JSContactFilter: typeof JSContactFilter;
    b2ContactID: typeof b2ContactID;
    b2ContactImpulse: typeof b2ContactImpulse;
    b2DestructionListener: typeof b2DestructionListener;
    b2DestructionListenerWrapper: typeof b2DestructionListenerWrapper;
    JSDestructionListener: typeof JSDestructionListener;
    b2DistanceJoint: typeof b2DistanceJoint;
    b2DistanceJointDef: typeof b2DistanceJointDef;
    b2Draw: typeof b2Draw;
    JSDraw: typeof JSDraw;
    b2FrictionJoint: typeof b2FrictionJoint;
    b2FrictionJointDef: typeof b2FrictionJointDef;
    b2GearJoint: typeof b2GearJoint;
    b2GearJointDef: typeof b2GearJointDef;
    b2JointEdge: typeof b2JointEdge;
    b2Manifold: typeof b2Manifold;
    b2ManifoldPoint: typeof b2ManifoldPoint;
    b2Mat22: typeof b2Mat22;
    b2Mat33: typeof b2Mat33;
    b2MouseJoint: typeof b2MouseJoint;
    b2MouseJointDef: typeof b2MouseJointDef;
    b2PolygonShape: typeof b2PolygonShape;
    b2PrismaticJoint: typeof b2PrismaticJoint;
    b2PrismaticJointDef: typeof b2PrismaticJointDef;
    b2Profile: typeof b2Profile;
    b2PulleyJoint: typeof b2PulleyJoint;
    b2PulleyJointDef: typeof b2PulleyJointDef;
    b2RayCastInput: typeof b2RayCastInput;
    b2RayCastOutput: typeof b2RayCastOutput;
    b2RevoluteJoint: typeof b2RevoluteJoint;
    b2RevoluteJointDef: typeof b2RevoluteJointDef;
    b2RopeJoint: typeof b2RopeJoint;
    b2RopeJointDef: typeof b2RopeJointDef;
    b2Rot: typeof b2Rot;
    b2WheelJoint: typeof b2WheelJoint;
    b2WheelJointDef: typeof b2WheelJointDef;
    b2MotorJoint: typeof b2MotorJoint;
    b2MotorJointDef: typeof b2MotorJointDef;
    b2ParticleColor: typeof b2ParticleColor;
    b2ParticleDef: typeof b2ParticleDef;
    b2ParticleGroupDef: typeof b2ParticleGroupDef;
    b2ParticleGroup: typeof b2ParticleGroup;
    b2ParticleSystemDef: typeof b2ParticleSystemDef;
    b2ParticleSystem: typeof b2ParticleSystem;
    b2ParticleContact: typeof b2ParticleContact;
    b2ParticleBodyContact: typeof b2ParticleBodyContact;
    b2ParticlePair: typeof b2ParticlePair;
    b2ParticleTriad: typeof b2ParticleTriad;
}
