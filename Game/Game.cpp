#include "Dawn/Dawn.h"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <vector>

#include <fmod.hpp>

static const float PI = 3.1415927;

static const float PHYSICS_TIMESTEP = 1.0f / 60.0f;
static const float BALL_SIZE = 0.2f;

static float SqrMagnitude(Dawn::Vec3 vec) {
    return vec.x * vec.x + vec.y * vec.y + vec.z * vec.z;
}

static float Magnitude(Dawn::Vec3 vec) {
    return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

static Dawn::Vec3 Norm(Dawn::Vec3 vec) {
	if (vec.x == 0 && vec.y == 0 && vec.z == 0) {
		return Dawn::Vec3(0, 0, 0);
	}
	return vec / Magnitude(vec);
}

static const Dawn::Vec3 ACCEL_GRAVITY(0, -1.5, 0);
static const float SLOP_MAXIMUM = 0.05;

static FMOD::System* fmod_system = nullptr;
static FMOD::Sound* clack_sound;
static FMOD::Sound* thump_sound;

static void playSound(FMOD::Sound* sound, float volume) {
	FMOD::Channel* channel;
	fmod_system->playSound(sound, nullptr, true, &channel);
	channel->setVolume(volume);
	channel->setPaused(false);
}

enum MatterType {
	WHITE_MATTER,
	RED_MATTER,
	BLUE_MATTER
};

struct Ball {
	MatterType matter;
	Dawn::Scene* scene;
	Dawn::Entity entity;
	//Dawn::TransformComponent& transform;
	float time_passed = 0;

	float mass;
	Dawn::Vec3 velocity;
	bool annihilating = false;
	Dawn::Vec3 explosion_impulse;

	Ball(Dawn::Scene* scene, MatterType matter, Dawn::Entity ent, float mass)
		: scene(scene), matter(matter), entity(ent), velocity(0, 0, 0), mass(mass), explosion_impulse(0, 0, 0) {
		//transform = scene.getComponent<Dawn::TransformComponent>(ent);
	}
	/*
	Ball(const Ball& ball) : scene(ball.scene) {
		matter = ball.matter;
		entity = ball.entity;
		time_passed = ball.time_passed;
		mass = ball.mass;
		velocity = ball.velocity;
		annihilating = ball.annihilating;
	}
	~Ball() {
		scene.deleteEntity(entity);
	}*/
    Dawn::Vec3 getPos() {
        auto& transform = scene->getComponent<Dawn::TransformComponent>(entity);
        return transform.position;
    }
	float kineticEnergy() {
		return 0.5 * mass * SqrMagnitude(velocity);
	}
	float potentialEnergy() {
		return Magnitude(ACCEL_GRAVITY) * mass * (getPos().y + 1.0);
	}
	void tickGravity() {
		auto& transform = scene->getComponent<Dawn::TransformComponent>(entity);
		Dawn::Vec3& pos = transform.position;
		
		float d_time = PHYSICS_TIMESTEP;

		time_passed += d_time;
		float wind;
		{
			float freq = 0.2;
			float amplitude = 0.5;
			wind = amplitude * sin(PI * freq * time_passed);
		}
		//std::cout << wind << std::endl;

		// Do explosion impulses here out of sheer laziness
		auto acceleration = ACCEL_GRAVITY + explosion_impulse;
		explosion_impulse = Dawn::Vec3(0, 0, 0);

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

		auto& transform = scene->getComponent<Dawn::TransformComponent>(entity);
		Dawn::Vec3& pos = transform.position;

        const float ELASTICITY = 0.9;

		// Bottom wall
		if ((pos.y - (BALL_SIZE / 2)) <= BOTTOM) {
			// Displacement
			float y_displace = BOTTOM - (pos.y - (BALL_SIZE / 2));
			Dawn::Vec3 n_displace = Norm(velocity) * -1;
			Dawn::Vec3 displace(n_displace.x / n_displace.y * y_displace, y_displace, 0);
			// Inelastic collision
			Dawn::Vec3 bounce_velocity(velocity.x * ELASTICITY, -velocity.y * ELASTICITY, 0);

			// Update
			pos = pos + displace;
			velocity = bounce_velocity;

			if (Magnitude(velocity) > 0.5) {
				playSound(thump_sound, fmin(fmax(0.0, Magnitude(velocity) - 0.5), 1.0));
			}
		}

		// Top wall
		if ((pos.y + (BALL_SIZE / 2)) >= 1.0f) {
			// Displacement
			float y_displace = 1.0f - (pos.y + (BALL_SIZE / 2));
			Dawn::Vec3 n_displace = Norm(velocity) * -1;
			Dawn::Vec3 displace(n_displace.x / n_displace.y * y_displace, y_displace, 0);
			// Inelastic collision
			Dawn::Vec3 bounce_velocity(velocity.x * ELASTICITY, -velocity.y * ELASTICITY, 0);

			// Update
			pos = pos + displace;
			velocity = bounce_velocity;

			if (Magnitude(velocity) > 0.5) {
				playSound(thump_sound, fmin(fmax(0.0, Magnitude(velocity) - 0.5), 1.0));
			}
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
			Dawn::Vec3 bounce_velocity(-velocity.x * ELASTICITY, velocity.y * ELASTICITY, 0);

			// Update
			pos = pos + displace;
			velocity = bounce_velocity;

			if (Magnitude(velocity) > 0.5) {
				playSound(thump_sound, fmin(fmax(0.0, Magnitude(velocity) - 0.5), 1.0));
			}
		}
	}
	std::tuple<Dawn::Vec3, float> collideBalls(int id, const std::vector<Ball>& balls) {
		auto& transform = scene->getComponent<Dawn::TransformComponent>(entity);
		Dawn::Vec3& pos = transform.position;

		Dawn::Vec3 total_acceleration(0, 0, 0);
		float total_energy_loss = 1.0;
		for (int i = 0; i < balls.size(); i++) {
			if (i == id) {
				continue;
			}
			auto& ball = balls[i];
			auto& ball_transform = scene->getComponent<Dawn::TransformComponent>(ball.entity);
			auto& ball_pos = ball_transform.position;

			float d_x = pos.x - ball_pos.x;
			float d_y = pos.y - ball_pos.y;
			float distance_squared = d_x * d_x + d_y * d_y;
			if (distance_squared > BALL_SIZE * BALL_SIZE) {
				continue;
			}

			// Do we annihilate?
			if ((matter == RED_MATTER  && ball.matter == BLUE_MATTER) ||
				(matter == BLUE_MATTER && ball.matter == RED_MATTER)) {
				annihilating = true;
			}

			// Overlap detected - add force
			if (Magnitude(velocity) > 0.5) {
				playSound(clack_sound, fmin(fmax(0.0, Magnitude(velocity) - 0.5), 1.0));
			}
			float force_constant = 0.5;
			if (distance_squared == 0) {
				distance_squared = 0.01;
			}
			float accel_magnitude = force_constant / (distance_squared * mass);
			Dawn::Vec3 force_normal = Norm(Dawn::Vec3(d_x, d_y, 0));
			if (force_normal.x == 0 && force_normal.y == 0) {
				force_normal = Dawn::Vec3(-1.0, 0, 0);
			}
			total_acceleration = total_acceleration + (force_normal * accel_magnitude);

			// Energy loss
			auto vel_normal = Normalize(velocity);
			auto ball_vel_normal = Norm(ball.velocity);
			float dot = Dawn::Dot(vel_normal, ball_vel_normal);
			float energy_loss = sqrt((PI - acos(dot)) / PI);
			if (isnan(energy_loss)) {
				continue;
			}
			total_energy_loss *= energy_loss;
		}

		return std::make_tuple(total_acceleration, total_energy_loss);
	}
	void cleanSlop() {
		if ((kineticEnergy() + potentialEnergy()) < SLOP_MAXIMUM) {
			// Oppose slop with a small force
			float amt = 0.9;
			Dawn::Vec3 slop_fighter = velocity * -1.0f * amt / PHYSICS_TIMESTEP;
			velocity = slop_fighter * PHYSICS_TIMESTEP + velocity;
		}
	}
};

static const Dawn::Vec4 BALL_COLORS[] = {
		Dawn::Vec4(1.0, 1.0, 1.0, 1.0),
		Dawn::Vec4(1.0, 0.0, 0.0, 1.0),
		Dawn::Vec4(0.0, 0.0, 1.0, 1.0)
};
static const float BALL_CHANCES[] = {
	0.25, 0.37, 0.38
};
static const size_t BALL_COLOR_COUNT = sizeof(BALL_COLORS) / sizeof(Dawn::Vec4);

static const float CURSOR_ALPHA = 0.3;
static const int BASE_CHANCE_POINTS[]   = {   20,   40,   40 };
static const int EXPECTED_PROPORTIONS[] = { 0.20, 0.40, 0.40 };

class Game : public Dawn::Application {
	std::vector<Ball> balls;
	Dawn::Texture ball_texture;
	Dawn::Scene scene;
	float countdown = 1.0;
	Dawn::Vec3 mouse_pos;
	MatterType next_ball = WHITE_MATTER;
	Dawn::Entity cursor;

	std::vector<Dawn::Texture> number_textures;
	std::vector<Dawn::Entity> number_entities;
	int last_score = -1;
	int highest_score = 0;
	bool reset_score = false;

	int ball_counts[3] = { 0, 0, 0 };

	//Dawn::Entity flash;
	//Dawn::Texture flash_texture;

public:
	MatterType whichBallNext() {
		int total_balls = ball_counts[0] + ball_counts[1] + ball_counts[2];
		if (total_balls < 5) {
			// Too small a sample for the complicated stuff, just pick randomly...
			float rand_value = (float)(rand() % 1000) / 1000;
			if (rand_value < 0.33) {
				return WHITE_MATTER;
			} else if (rand_value < 0.66) {
				return RED_MATTER;
			} else {
				return BLUE_MATTER;
			}
		}
		float proportions[3] = { 0.0f, 0.0f, 0.0f };
		for (int i = 0; i < 3; i++) {
			proportions[i] = (float) ball_counts[i] / (float) total_balls;
		}
		float deltas[3] = { 0.0f, 0.0f, 0.0f };
		for (int i = 0; i < 3; i++) {
			deltas[i] = EXPECTED_PROPORTIONS[i] - proportions[i];
		}
		int point_deltas[3] = { 0, 0, 0 };
		for (int i = 0; i < 3; i++) {
			// 2% -> 1 chance point
			point_deltas[i] = trunc(deltas[i] * 50.0f);
		}
		int chance_points[3] = { 0, 0, 0 };
		for (int i = 0; i < 3; i++) {
			chance_points[i] = BASE_CHANCE_POINTS[i] + point_deltas[i];
			if (chance_points[i] < 0) {
				chance_points[i] = 0;
			}
		}
		int total_chance_points = 0;
		for (int i = 0; i < 3; i++) {
			total_chance_points += chance_points[i];
		}
		// Use chance points to randomly select next ball
		//std::cout << total_chance_points << "|" << chance_points[0] << " " << chance_points[1] << " " << chance_points[2] << std::endl;
		float white_boundary = (float)chance_points[0] / (float)total_chance_points;
		float red_boundary = white_boundary + ((float)chance_points[1] / (float)total_chance_points);
		float rand_value = (float)(rand() % 1000) / 1000;
		//std::cout << rand_value << " " << white_boundary << " " << red_boundary << std::endl;
		if (rand_value < white_boundary) {
			return WHITE_MATTER;
		} else if (rand_value < red_boundary) {
			return RED_MATTER;
		} else {
			return BLUE_MATTER;
		}
	}
	int score() {
		int red  = ball_counts[1];
		int blue = ball_counts[2];
		if (red > blue) {
			return blue;
		} else {
			return red;
		}
	}
	void addBall(MatterType type, Dawn::Vec3 pos) {
		Dawn::Entity ball_entity;
		ball_entity = scene.addEntity();
		scene.addComponent<Dawn::TransformComponent>(ball_entity);
		auto& transform_component = scene.getComponent<Dawn::TransformComponent>(ball_entity);
		/*
		transform_component.position = Dawn::Vec3(
			(2.0f / (k + 1)) * (x + 1) - 1.0f,
			(2.0f / (k + 1)) * (y + 1) - 1.0f,
			(y + x * 5) * -0.1
		);*/
		transform_component.position = pos;
		transform_component.scale = Dawn::Vec3(BALL_SIZE, BALL_SIZE, 1);

		scene.addComponent<Dawn::SpriteRendererComponent>(ball_entity);
		auto& sprite_component = scene.getComponent<Dawn::SpriteRendererComponent>(ball_entity);
		sprite_component.texture = &ball_texture;

		//MatterType matter = (MatterType)((x + y) % 3);
		MatterType matter = type;
		sprite_component.color = BALL_COLORS[((int)matter) % 3];

		Ball ball(&scene, matter, ball_entity, 0.5);
		//ball.velocity.x = 2.0f - 4.0f * (float)(rand() % 100) / 100.0f;
		const float START_VELOCITY_SCALE = 0.2f;
		ball.velocity.x = pos.x * -1.0f * START_VELOCITY_SCALE;
		balls.insert(balls.begin(), ball);
		ball_counts[(int)matter]++;
	}
	void onClick(const Dawn::Event& evt) {
		if (evt.getType() != Dawn::EventType::MousePressed) {
			return;
		}
		const auto& mouse_click = (const Dawn::MousePressedEvent&) evt;
		addBall(next_ball, mouse_pos);
		/*
		float chance = (float)(rand() % 1000) / 1000.0f;
		MatterType matter;
		if (chance < 0.25) {
			matter = WHITE_MATTER;
		}
		else if (chance < 0.62) {
			matter = RED_MATTER;
		}
		else {
			matter = BLUE_MATTER;
		}
		next_ball = matter;*/
		next_ball = whichBallNext();
		auto& sprite_component = scene.getComponent<Dawn::SpriteRendererComponent>(cursor);
		sprite_component.texture = &ball_texture;
		sprite_component.color = BALL_COLORS[next_ball];
		sprite_component.color.w = CURSOR_ALPHA;
		//next_ball = (MatterType) (((int) next_ball + 1) % 3);
	}
	void onMouseMove(const Dawn::Event& evt) {
		if (evt.getType() != Dawn::EventType::MouseMoved) {
			return;
		}
		const auto& mouse_move = (const Dawn::MouseMovedEvent&)evt;
		mouse_pos.x = -1.0f + 2.0f * ((float) mouse_move.getX() / getWindow().getWidth());
		mouse_pos.y = +1.0f - 2.0f * ((float) mouse_move.getY() / getWindow().getHeight());
		mouse_pos.y = fmax(mouse_pos.y, -0.1);
		auto& cursor_transform = scene.getComponent<Dawn::TransformComponent>(cursor);
		cursor_transform.position = Dawn::Vec3(mouse_pos.x, mouse_pos.y, 0);
	}
	void onKeyPress(const Dawn::Event& evt) {
		if (evt.getType() != Dawn::EventType::KeyPressed) {
			return;
		}
		const auto& key_press = (const Dawn::KeyPressedEvent&)evt;
		if (key_press.getKeyCode() == Dawn::KeyCode::R) {
			/*
			std::vector<Ball> balls;
			Dawn::Texture ball_texture;
			Dawn::Scene scene;
			float countdown = 1.0;
			Dawn::Vec3 mouse_pos;
			MatterType next_ball = WHITE_MATTER;
			Dawn::Entity cursor;

			std::vector<Dawn::Texture> number_textures;
			std::vector<Dawn::Entity> number_entities;
			int last_score = -1;
			int highest_score = 0;

			int ball_counts[3] = { 0, 0, 0 };*/
			for (auto& ball : balls) {
				scene.deleteEntity(ball.entity);
			}
			balls.clear();
			next_ball = WHITE_MATTER;
			last_score = 0;
			highest_score = 0;
			ball_counts[0] = 0;
			ball_counts[1] = 0;
			ball_counts[2] = 0;
			reset_score = true;
		}
	}
	Game() : mouse_pos(0, 0, 0) {
		// FMOD
		{
			FMOD::System_Create(&fmod_system);
			fmod_system->init(512, FMOD_INIT_NORMAL, nullptr);
			fmod_system->createSound("clack.ogg", FMOD_DEFAULT, nullptr, &clack_sound);
			fmod_system->createSound("thump.ogg", FMOD_DEFAULT, nullptr, &thump_sound);
		}

		// No framerate limit
		glfwSwapInterval(1);

		// Events
		Dawn::EventHandler::Listen(Dawn::EventType::MousePressed, BIND_EVENT_MEMBER_FN(Game::onClick));
		Dawn::EventHandler::Listen(Dawn::EventType::MouseMoved,   BIND_EVENT_MEMBER_FN(Game::onMouseMove));
		Dawn::EventHandler::Listen(Dawn::EventType::KeyPressed,   BIND_EVENT_MEMBER_FN(Game::onKeyPress));

		// Set window size
		getWindow().setHeight(800);
		getWindow().setWidth(800);

		// Load texture
		ball_texture.loadFromFile("Ball.png");

		// Number textures
		number_textures.resize(10);
		for (int i = 0; i < 10; i++) {
			char buf[64];
			sprintf_s(buf, "numbers/%d.png", i);
			number_textures[i].loadFromFile(buf);
		}

		// Flash
#if 0
		{
			flash_texture.loadFromFile("background.png");
			flash = scene.addEntity();
			scene.addComponent<Dawn::TransformComponent>(flash);
			scene.addComponent<Dawn::SpriteRendererComponent>(flash);
			auto& transform = scene.getComponent<Dawn::TransformComponent>(flash);
			auto& sprite = scene.getComponent<Dawn::SpriteRendererComponent>(flash);
			transform.scale = Dawn::Vec3(2.0, 2.0, 1.0);
			transform.position = Dawn::Vec3(0, 0, -1.0);
			sprite.texture = &flash_texture;
			sprite.color = Dawn::Vec4(1.0, 1.0, 1.0, 1.0);
		}
#endif

		// Sounds

		// Cursor
		cursor = scene.addEntity();
		scene.addComponent<Dawn::SpriteRendererComponent>(cursor);
		auto& sprite_component = scene.getComponent<Dawn::SpriteRendererComponent>(cursor);
		sprite_component.texture = &ball_texture;
		sprite_component.color = Dawn::Vec4(1.0, 1.0, 1.0, CURSOR_ALPHA);
		auto& transform_component = scene.getComponent<Dawn::TransformComponent>(cursor);
		transform_component.scale = Dawn::Vec3(BALL_SIZE, BALL_SIZE, 0);

		// Set up balls
		/*
		int k = 8;
		for (int y = 0; y < k; y++) {
			for (int x = 0; x < k; x++) {
				
			}
		}*/
	}
	void onUpdate() override {
		fmod_system->update();

#if 0
		{
			auto& flash_sprite = scene.getComponent<Dawn::SpriteRendererComponent>(flash);
			if (flash_sprite.color.w > 0.0) {
				flash_sprite.color.w -= Dawn::Time::deltaTime * 4.0f;
			}
			if (flash_sprite.color.w <= 0.0) {
				flash_sprite.color.w = 0.0;
			}
		}
#endif

		//std::cout << Dawn::Time::deltaTime << std::endl;
		//std::cout << "W " << ball_counts[WHITE_MATTER] << "; B " << ball_counts[BLUE_MATTER] << "; R" << ball_counts[RED_MATTER] << "\r" << std::flush;

		if (countdown >= 0.0) {
			countdown -= Dawn::Time::deltaTime;
			scene.onUpdate();
			return;
		}

        // Stats
#if 0
        {
            float total_momentum = 0.0;
            float total_kinetic_energy = 0.0;
            float total_potential_energy = 0.0;
            for (auto& ball : balls) {
                total_momentum += ball.mass * Magnitude(ball.velocity);
				total_kinetic_energy += ball.kineticEnergy();
				total_potential_energy += ball.potentialEnergy();
                std::cout 
					<< std::fixed << std::setprecision(3) <<    "p: " << total_momentum 
					<< std::fixed << std::setprecision(3) << " | E: " << total_kinetic_energy + total_potential_energy
					<< std::fixed << std::setprecision(3) << " | K: " << total_kinetic_energy
					<< std::fixed << std::setprecision(3) << " | U: " << total_potential_energy
					<< "\r" << std::flush;
            }
        }
#endif
        
		// Show score
		if (score() != last_score || reset_score) {
			reset_score = false;
			last_score = score();
			if (last_score > highest_score) {
				highest_score = last_score;
			}
			for (auto& ent : number_entities) {
				scene.deleteEntity(ent);
			}
			number_entities.clear();
			{
				char characters[512];
				sprintf_s(characters, "%d", last_score);
				int len = strlen(characters);
				for (int i = len - 1; i >= 0; i--) {
					Dawn::Entity ent = scene.addEntity();

					scene.addComponent<Dawn::TransformComponent>(ent);
					auto& transform = scene.getComponent<Dawn::TransformComponent>(ent);
					Dawn::Vec3 base_position(0.9, 0.9, 0);
					transform.position = Dawn::Vec3(base_position.x - (len - i - 1) * 0.08, base_position.y, 0);
					transform.scale = Dawn::Vec3(0.1, 0.1, 1.0);

					scene.addComponent<Dawn::SpriteRendererComponent>(ent);
					auto& sprite = scene.getComponent<Dawn::SpriteRendererComponent>(ent);
					sprite.texture = &number_textures[characters[i] - '0'];
					//sprite.texture = &number_textures[0];
					sprite.color = Dawn::Vec4(1.0, 1.0, 1.0, 1.0);

					number_entities.push_back(ent);
				}
			}
			{
				char characters[512];
				sprintf_s(characters, "%d", highest_score);
				int len = strlen(characters);
				for (int i = len - 1; i >= 0; i--) {
					Dawn::Entity ent = scene.addEntity();

					scene.addComponent<Dawn::TransformComponent>(ent);
					auto& transform = scene.getComponent<Dawn::TransformComponent>(ent);
					Dawn::Vec3 base_position(0.9, 0.9, 0);
					transform.position = Dawn::Vec3(base_position.x - (len - i - 1) * 0.08, base_position.y - 0.15, 0);
					transform.scale = Dawn::Vec3(0.1, 0.1, 1.0);

					scene.addComponent<Dawn::SpriteRendererComponent>(ent);
					auto& sprite = scene.getComponent<Dawn::SpriteRendererComponent>(ent);
					sprite.texture = &number_textures[characters[i] - '0'];
					//sprite.texture = &number_textures[0];
					sprite.color = Dawn::Vec4(1.0, 1.0, 1.0, 1.0);

					number_entities.push_back(ent);
				}
			}
		}

		// Apply gravity and wall collisions
        std::for_each(balls.begin(), balls.end(), std::mem_fn(&Ball::tickGravity));
		std::for_each(balls.begin(), balls.end(), std::mem_fn(&Ball::collideWalls));

		// Collide balls and detect annihilations
		std::vector<std::tuple<Dawn::Vec3, float>> modifications;
		modifications.reserve(balls.size());
		for (int i = 0; i < balls.size(); i++) {
			modifications.push_back(balls[i].collideBalls(i, balls));
		}
		for (int i = 0; i < balls.size(); i++) {
			const auto& acceleration = std::get<0>(modifications[i]);
			// Workaround for NaN issue...
			if (isnan(acceleration.x) || isnan(acceleration.y) || isnan(acceleration.z)) {
				std::cout << "NaN detected!" << std::endl;
				continue;
			}
			float energy_loss = std::get<1>(modifications[i]);
			// Apply accelerations
			balls[i].velocity = acceleration * PHYSICS_TIMESTEP + balls[i].velocity;
			// Apply energy loss
			balls[i].velocity = balls[i].velocity * energy_loss;
		}

		// Annihilate pairs
		std::vector<Dawn::Vec3> explosions;
		for (auto it = balls.begin(); it != balls.end();) {
			if (it->annihilating) {
				/*
				{
					auto& flash_sprite = scene.getComponent<Dawn::SpriteRendererComponent>(flash);
					flash_sprite.color.w = 1.0;
				}*/
				ball_counts[(int)it->matter]--;
				explosions.push_back(it->getPos());
				scene.deleteEntity(it->entity);
				it = balls.erase(it);
			} else {
				it++;
			}
		}

		// Clean up physics slop
		std::for_each(balls.begin(), balls.end(), std::mem_fn(&Ball::cleanSlop));

		// Add explosion impulses for next frame
		for (auto& ball : balls) {
			Dawn::Vec3 total_accel(0, 0, 0);
			for (auto& pos : explosions) {
				auto difference = ball.getPos() - pos;
				auto force_normal = Normalize(difference);
				auto distance_sqr = SqrMagnitude(difference);
				if (distance_sqr == 0) {
					continue;
				}
				float force_scale = 7.0f;
				auto force = force_scale / distance_sqr;
				auto acceleration = force / ball.mass;
				total_accel = total_accel + (force_normal * acceleration);
			}
			ball.explosion_impulse = total_accel;
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
