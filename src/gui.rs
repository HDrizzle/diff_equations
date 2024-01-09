// Uses bevy for GUI
use bevy::{prelude::*, core::Zeroable};
use crate::prelude::*;

struct MainPlugin;

// Components
#[derive(Component)]
pub struct CameraComponent;

// Systems
fn render_setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>
) {
    commands.spawn((
		Camera3dBundle {
			transform: Transform::from_xyz(50.0, 15.0, 50.0).looking_at(Vec3::new(0., 0., 0.), Vec3::Y),
			..default()
		},
		CameraComponent
	));
    commands.spawn(
        PbrBundle{
            mesh: meshes.add(shape::Box::from_corners(Vec3::zeroed(), Vec3::splat(5.0)).into()),
            ..default()
        }
    );
}

impl Plugin for MainPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, render_setup);
    }
}

pub fn main<T: StaticDifferentiator + Clone + Send + Sync + 'static>(stepper: Stepper<T>) {
    let mut app = App::new();
    app.add_plugins((
        DefaultPlugins.set(WindowPlugin {
			primary_window: Some(Window {
				title: APP_NAME.to_string(),
				..Default::default()
			}),
			..Default::default()
		}),
        MainPlugin
    ));
    app.insert_resource(stepper);
    println!("Starting bevy app");
    app.run();
}