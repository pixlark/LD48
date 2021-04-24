#include "Dawn/Dawn.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>

static const float PHYSICS_TIMESTEP = 1.0f / 120.0f;
static const float BALL_SIZE = 0.1f;

static Dawn::Vec3 Norm(Dawn::Vec3 vec) {
	if (vec.x == 0 && vec.y == 0 && vec.z == 0) {
		return Dawn::Vec3(0, 0, 0);
	}
	return vec / sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

struct Ball {
	Dawn::Scene& scene;
	Dawn::Entity entity;
	//Dawn::TransformComponent& transform;
	float time_passed = 0;

	float mass;
	Dawn::Vec3 velocity;

	Ball(Dawn::Scene& scene, Dawn::Entity ent, float mass) 
		: scene(scene), entity(ent), velocity(0, 0, 0), mass(mass) {
		//transform = scene.getComponent<Dawn::TransformComponent>(ent);
	}
	void tickGravity() {
		auto& transform = scene.getComponent<Dawn::TransformComponent>(entity);
		Dawn::Vec3& pos = transform.position;
		
		float d_time = PHYSICS_TIMESTEP;

		time_passed += d_time;
		float wind;
		{
			float pi = 3.14159;
			float freq = 0.2;
			float amplitude = 0.5;
			wind = amplitude * sin(pi * freq * time_passed);
		}
		//std::cout << wind << std::endl;
		Dawn::Vec3 acceleration(0, -1, 0);

		Dawn::Vec3 d_velocity =
			(acceleration * d_time);

		Dawn::Vec3 d_position =
			(velocity * d_time) + (acceleration * 0.5 * d_time * d_time);

		velocity = velocity + d_velocity;
		pos = pos + d_position;
	}
	void collideWalls() {
		const float BOTTOM = -1.0f;
		const float LEFT   = -1.0f;
		const float RIGHT  = +1.0f;

		auto& transform = scene.getComponent<Dawn::TransformComponent>(entity);
		Dawn::Vec3& pos = transform.position;

		// Bottom wall
		if ((pos.y - (BALL_SIZE / 2)) <= BOTTOM) {
			// Displacement
			float y_displace = BOTTOM - (pos.y - (BALL_SIZE / 2));
			Dawn::Vec3 n_displace = Norm(velocity) * -1;
			Dawn::Vec3 displace(n_displace.x / n_displace.y * y_displace, y_displace, 0);
			// Inelastic collision
			Dawn::Vec3 bounce_velocity(velocity.x * 0.9, -velocity.y * 0.9, 0);

			// Update
			pos = pos + displace;
			velocity = bounce_velocity;
		}

		// Side walls
		if ((pos.x - (BALL_SIZE / 2)) <= LEFT ||
			(pos.x + (BALL_SIZE / 2)) >= RIGHT) {
			// Displacement
			float x_displace = ((pos.x - (BALL_SIZE / 2)) <= LEFT)
				? (LEFT - (pos.x - (BALL_SIZE / 2)))
				: (RIGHT - (pos.x + (BALL_SIZE / 2)));
			Dawn::Vec3 n_displace = Norm(velocity) * -1;
			Dawn::Vec3 displace(x_displace, n_displace.y / n_displace.x * x_displace, 0);
			// Inelastic collision
			Dawn::Vec3 bounce_velocity(-velocity.x * 0.9, velocity.y * 0.9, 0);

			// Update
			pos = pos + displace;
			velocity = bounce_velocity;
		}
	}
	std::pair<Dawn::Vec3, float> collideBalls(int id, const std::vector<Ball>& balls) {
		auto& transform = scene.getComponent<Dawn::TransformComponent>(entity);
		Dawn::Vec3& pos = transform.position;

		Dawn::Vec3 total_acceleration(0, 0, 0);
		float total_energy_loss = 1.0;
		for (int i = 0; i < balls.size(); i++) {
			if (i == id) {
				continue;
			}
			auto& ball = balls[i];
			auto& ball_transform = scene.getComponent<Dawn::TransformComponent>(ball.entity);
			auto& ball_pos = ball_transform.position;

			float d_x = pos.x - ball_pos.x;
			float d_y = pos.y - ball_pos.y;
			float distance_squared = d_x * d_x + d_y * d_y;
			if (distance_squared > BALL_SIZE * BALL_SIZE) {
				continue;
			}

			// Overlap detected - add force
			float force_constant = 0.3;
			float accel_magnitude = force_constant / (distance_squared * mass);
			Dawn::Vec3 force_normal = Norm(Dawn::Vec3(d_x, d_y, 0));
			total_acceleration = total_acceleration + (force_normal * accel_magnitude);

			// Energy loss
			auto vel_normal = Normalize(velocity);
			auto ball_vel_normal = Norm(ball.velocity);
			float dot = Dawn::Dot(vel_normal, ball_vel_normal);
			float energy_loss = (acos(dot) / 3.14159);
			total_energy_loss *= energy_loss;
		}

		return std::make_pair(total_acceleration, total_energy_loss);
	}
};

static const Dawn::Vec4 BALL_COLORS[] = {
		Dawn::Vec4(1.0, 1.0, 1.0, 1.0),
		Dawn::Vec4(1.0, 0.0, 0.0, 1.0),
		Dawn::Vec4(0.0, 1.0, 0.0, 1.0),
		Dawn::Vec4(0.0, 0.0, 1.0, 1.0)
};
static const size_t BALL_COLOR_COUNT = sizeof(BALL_COLORS) / sizeof(Dawn::Vec4);

class Game : public Dawn::Application {
	std::vector<Ball> balls;
	Dawn::Texture ball_texture;
	Dawn::Scene scene;
	float countdown = 1.0;
	
public:
	Game() {
		// No framerate limit
		glfwSwapInterval(0);

		// Set window size
		getWindow().setHeight(800);
		getWindow().setWidth(800);

		// Load texture
		ball_texture.loadFromFile("Ball.png");

		// Set up balls
		for (int y = 0; y < 5; y++) {
			for (int x = 0; x < 5; x++) {
				Dawn::Entity ball_entity;
				ball_entity = scene.addEntity();
				scene.addComponent<Dawn::TransformComponent>(ball_entity);
				auto& transform_component = scene.getComponent<Dawn::TransformComponent>(ball_entity);
				transform_component.position = Dawn::Vec3(
					(float)(x - 2) / 3.0f,
					(float)(y - 2) / 3.0f,
					0
				);
				transform_component.scale = Dawn::Vec3(BALL_SIZE, BALL_SIZE, 1);

				scene.addComponent<Dawn::SpriteRendererComponent>(ball_entity);
				auto& sprite_component = scene.getComponent<Dawn::SpriteRendererComponent>(ball_entity);
				sprite_component.texture = &ball_texture;
				sprite_component.color = BALL_COLORS[(x + y) % BALL_COLOR_COUNT];

				Ball ball(scene, ball_entity, 1.0);
				ball.velocity.x = 0.5f - (float)(rand() % 100) / 100.0f;
				ball.mass = 0.2 * y;
				balls.push_back(ball);
			}
		}


	}
	void onUpdate() override {
		//std::cout << Dawn::Time::deltaTime << std::endl;

		if (countdown >= 0.0) {
			countdown -= Dawn::Time::deltaTime;
			scene.onUpdate();
			return;
		}

		std::for_each(balls.begin(), balls.end(), std::mem_fn(&Ball::tickGravity));
		std::for_each(balls.begin(), balls.end(), std::mem_fn(&Ball::collideWalls));
		std::vector<std::pair<Dawn::Vec3, float>> modifications;
		modifications.reserve(balls.size());
		for (int i = 0; i < balls.size(); i++) {
			modifications.push_back(balls[i].collideBalls(i, balls));
		}
		for (int i = 0; i < balls.size(); i++) {
			auto acceleration = modifications[i].first;
			float energy_loss = modifications[i].second;
			// Apply accelerations
			balls[i].velocity = acceleration * PHYSICS_TIMESTEP + balls[i].velocity;
			// Apply energy loss
			balls[i].velocity = balls[i].velocity * energy_loss;
			std::cout << balls[i].velocity.x << balls[i].velocity.y << balls[i].velocity.z << std::endl;
		}

		scene.onUpdate();
	}
	void onClose() override {
		// ...
	}
};

int main() {
	srand(0);

	Game game;
	game.start();
}
