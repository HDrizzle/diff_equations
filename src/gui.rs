// Uses bevy for GUI
use bevy::prelude::*;
use crate::prelude::*;

struct MainPlugin;

// Components
#[derive(Component)]
pub struct CameraComponent;

// Systems
fn render_setup(mut commands: Commands) {
    commands.spawn((
		Camera3dBundle {
			transform: Transform::from_xyz(50.0, 10.0, 0.0).looking_at(Vec3::new(50., 0., 50.), Vec3::Y),
			..default()
		},
		CameraComponent
	));
}

impl Plugin for MainPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, render_setup);
    }
}

pub fn main<const N: usize, T: StaticDifferentiator<N> + Clone>(stepper: Stepper<N, T>) {
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
    println!("Starting bevy app");
    app.run();
}