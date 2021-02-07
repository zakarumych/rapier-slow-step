use rapier3d::{
    dynamics::{IntegrationParameters, JointSet, RigidBodyBuilder, RigidBodySet},
    geometry::{BroadPhase, ColliderBuilder, ColliderSet, NarrowPhase},
    na::Vector3,
    pipeline::PhysicsPipeline,
};

fn main() {
    // Here the gravity is -9.81 along the y axis.
    let mut pipeline = PhysicsPipeline::new();
    let gravity = Vector3::new(0.0, -9.81, 0.0);
    let integration_parameters = IntegrationParameters::default();
    let mut broad_phase = BroadPhase::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let mut joints = JointSet::new();
    // We ignore contact events for now.
    let event_handler = ();

    let cone = ColliderBuilder::cone(1.0, 1.0).build();

    for _ in 0..10 {
        let body = bodies.insert(RigidBodyBuilder::new_dynamic().build());
        colliders.insert(cone.clone(), body, &mut bodies);
    }

    pipeline.step(
        &gravity,
        &integration_parameters,
        &mut broad_phase,
        &mut narrow_phase,
        &mut bodies,
        &mut colliders,
        &mut joints,
        None,
        None,
        &event_handler,
    );
}
