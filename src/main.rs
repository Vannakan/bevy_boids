use std::{borrow::BorrowMut};

use bevy::{prelude::*, window::PrimaryWindow, app::PluginGroupBuilder};
use rand::Rng;

fn main() {
    App::new()
    .add_plugins((build_default_plugins(),BoidPlugin))
    .add_systems(Startup, setup)
    .run();
}

const WINDOW_HEIGHT: f32 = 1200.0;
const WINDOW_WIDTH: f32 = 1600.0;

// move cam to camera mod
pub fn setup(mut commands: Commands){
    commands.spawn((Camera2dBundle {
        ..default()
    }));
}

pub fn build_default_plugins() -> PluginGroupBuilder {
    DefaultPlugins
        .set(ImagePlugin::default_nearest())
        .set(WindowPlugin {
            primary_window: Some(Window {
                title: "boids".to_string(),
                resolution: (WINDOW_WIDTH, WINDOW_HEIGHT).into(),
                resizable: false,
                ..default()
            }),
            ..default()
        },
    )
        .build()
}

#[derive(Component)]
pub struct Velocity(Vec3);
#[derive(Component)]
pub struct Boid;
#[derive(Component)]
pub struct Seek;
#[derive(Component)]
pub struct Target(Vec3);

#[derive(Component)]
pub struct Neighbours(Vec<(Entity, Vec3, Vec3)>); // vel, pos

pub struct BoidPlugin;

const DISTANCE: f32 = 300.0;

pub fn boid_movement_system(
    mut movement_query: Query<(&mut Transform, &mut Velocity)>, time: Res<Time>,){
    for movement in movement_query.iter_mut()
    {
        let (mut transform, mut velocity) = movement;

        if(velocity.0 == Vec3::default()) { continue; }
        transform.translation += ((velocity.0.normalize() * 2.0) );
    
        if velocity.0.x >= -0.1 && velocity.0.x <= 0.1 && velocity.0.y <= 0.1 && velocity.0.y >= -0.1
        {
            velocity.0 = Vec3::default();
        }
        else {
            velocity.0 = velocity.0.lerp(Vec3::default(), 0.1)
        }
    }
}

impl Plugin for BoidPlugin {
    fn build(&self, app: &mut App) {

        app.add_systems(Startup, spawn_boids)
        .add_systems(Update, boid_movement_system)
        .add_systems(Update, boid_seek_system)
        .add_systems(Update, boid_avoidance)
        .add_systems(Update, boid_find_neighbours)
       .add_systems(Update, set_boid_target);
    }
}

const BOIDS: i32 = 5;

pub fn set_boid_target(mut commands: Commands, input: Res<Input<MouseButton>>, query: Query<Entity, With<Boid>>, q_windows: Query<&Window, With<PrimaryWindow>>,
    camera_q: Query<(&Camera, &GlobalTransform)>) {
    if query.is_empty() { return; }
    if input.just_released(MouseButton::Left){
        let (camera, camera_transform) = camera_q.single();
        let mouse_position =  q_windows.single().cursor_position().unwrap();
        let position =Vec3::from((camera.viewport_to_world_2d(camera_transform, mouse_position).unwrap(), 1.0));
        for entity in query.iter() {
            let mut e = commands.get_entity(entity).unwrap();
            e.remove::<Target>();
            e.insert(Target(position));
        }
    }
}


pub fn set_random_boid_target(mut commands: Commands, query: Query<(Entity, &Target, &Transform), With<Boid>>){
    let mut rand = rand::thread_rng();
    for entity in query.iter() {
        if entity.2.translation.distance(entity.1.0) > 25.0 { continue;  }
        let mut e = commands.get_entity(entity.0).unwrap();
        e.remove::<Target>();
        e.insert(Target(Vec3{x: rand.gen_range(-DISTANCE..=DISTANCE), y: rand.gen_range(-DISTANCE..=DISTANCE),  z: 0.0}));
    }
}

pub fn boid_avoidance(mut bandit_query: Query<(&mut Transform, &mut Velocity)>){
    let mut combinations = bandit_query.iter_combinations_mut();
    while let Some([mut t1, t2]) = combinations.fetch_next()
    {
        if t1.0.translation.distance(t2.0.translation) > 300.0 { continue;}
        let t1_pos = t1.0.translation;
        let t2_pos = t2.0.translation;
        t1.borrow_mut().1.0 += calc_avoidance(t1_pos,t2_pos)
    }
}

pub fn boid_find_neighbours(mut n_query: Query<(Entity, &mut Neighbours)>, mut query: Query<(Entity, &Transform, &mut Velocity), With<Boid>>){
    // for mut n_entity in n_query.iter_mut(){
    //     n_entity.1.0.clear();
        for entity in query.iter(){
            let mut neighbours = n_query.get_mut(entity.0).unwrap();
            neighbours.1.0.clear();
            for other_entity in query.iter(){
                if other_entity.0 != entity.0 {
                    if other_entity.1.translation.distance(entity.1.translation) > 300.0 { continue; }
                    neighbours.1.0.push((other_entity.0, other_entity.2.0, other_entity.1.translation));
                }
            }
       // }
    }


    for mut entity in query.iter_mut() {
        let mut neighbours = n_query.get_mut(entity.0).unwrap();
        if neighbours.1.0.len() <= 0 { continue; }


        // Alignment
        let mut vel = Vec3::from([0.0, 0.0, 0.0]);
        for neighbour in neighbours.1.0.iter() {
            //println!("neighbour vec: {}", neighbour.1);
             vel += neighbour.1;
        }
        if vel != Vec3::default(){
            entity.2.0 +=  (vel / neighbours.1.0.len() as f32).normalize();
        }
      
        // Cohesion
        let mut vel2 = Vec3::from([0.0, 0.0, 0.0]);
        for neighbour in neighbours.1.0.iter() {
            vel2 += neighbour.2; 

            
        }

        if(vel2 != Vec3::default()){
            vel2 /= neighbours.1.0.len() as f32;
            vel2 = Vec3::from([vel2.x - entity.1.translation.x, vel2.y - entity.1.translation.y, 0.0]);
    
            entity.2.0 += vel2.normalize();
        } 
    }
}

// fn calc_cohesion(){

// }

// fn calc_separation() {

// }

// fn calc_alignment() {

// }

pub fn calc_avoidance(a: Vec3, b: Vec3) -> Vec3{
    // Overlapping - Addresses issue where direction is NaN whenever the vec3s are the same
    if a == b {
        let mut rand = rand::thread_rng();
        return Vec3{x: rand.gen_range(-1.0..=1.0) * 25.0, y: rand.gen_range(-1.0..=1.0) * 25.0, z: 0.0}
    }
    let direction = (b - a).normalize() * 25.0;
    Vec3{x: direction.x , y: direction.y  , z: 0.0} *-1.0
}

pub fn visibility(query: Query<&ComputedVisibility>){
    for e in query.iter(){
        println!("{}", e.is_visible_in_view())
    }
}

pub fn boid_seek_system(mut query: Query<(&Transform, &mut Velocity, &Target), (With<Seek>, With<Boid>)>)
{
    if query.is_empty() { return; }

    for (transform, mut velocity, target) in query.iter_mut() {
        let direction = (transform.translation - target.0).normalize();
        velocity.0 -= Vec3{x: direction.x * 25.0, y: direction.y * 25.0, z: 0.0} ;
    }
}

const SPAWN_RANGE: f32 = 500.0;

fn spawn_boids(mut commands: Commands, mut asset_server: ResMut<AssetServer>){

    let mut rand = rand::thread_rng();
    for _ in 0..=BOIDS{
        let pos = Vec3{x: rand.gen_range(-SPAWN_RANGE..=SPAWN_RANGE), y: rand.gen_range(-SPAWN_RANGE..=SPAWN_RANGE), z: 0.0};
        spawn_boid(&mut commands, &mut asset_server, pos);
    }
  
}

fn spawn_boid(commands: &mut Commands, asset_server: &mut ResMut<AssetServer>, position: Vec3 ){
   let mut rand = rand::thread_rng();
    let texture = asset_server.load("boid.png");
    let _ = commands.spawn((SpriteBundle {
        sprite: Sprite {
          //  custom_size: Some(Vec2::new(100.0, 100.0)),
            ..default()
        },
        texture,
        transform: Transform{
            translation: position,
            ..default()
        },
        ..default()
    },
   Target(Vec3{x: rand.gen_range(-DISTANCE..=DISTANCE), y: rand.gen_range(-DISTANCE..=DISTANCE),  z: 0.0}), // give random target at the beginning -- temp
   Boid,
    Neighbours(Vec::default()),
    Seek,
    Velocity(Vec3::default()))).id();
}
