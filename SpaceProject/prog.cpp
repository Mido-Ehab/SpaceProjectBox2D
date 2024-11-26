#include <SFML/Graphics.hpp>
#include <box2d/box2d.h>
#include <set>
#include <cmath>
#include <vector>
#include <iostream>



// Constants
const float SCALE = 30.0f;              // Pixels per Box2D unit
const float G = 200.0f;                // Gravitational constant
const float DECAY_MULTIPLIER = 0.002f; // Decay adjustment factor

class MyContactListener : public b2ContactListener {
public:
    std::set<b2Body*> bodiesToDestroy;

    void BeginContact(b2Contact* contact) override {
        b2Body* bodyA = contact->GetFixtureA()->GetBody();
        b2Body* bodyB = contact->GetFixtureB()->GetBody();

        void* userDataA = reinterpret_cast<void*>(bodyA->GetUserData().pointer);
        void* userDataB = reinterpret_cast<void*>(bodyB->GetUserData().pointer);

        if ((userDataA && strcmp((char*)userDataA, "sun") == 0) ||
            (userDataB && strcmp((char*)userDataB, "sun") == 0)) {
            b2Body* target = (userDataA && strcmp((char*)userDataA, "sun") == 0) ? bodyB : bodyA;
            bodiesToDestroy.insert(target);
        }
    }
};

int main() {
    // SFML Setup
    sf::RenderWindow window(sf::VideoMode(800, 600), "Sun-Based Gravity with Orbit Decay");
    window.setFramerateLimit(60);


    // Load Textures
    sf::Texture spaceTexture;
    if (!spaceTexture.loadFromFile("Space.PNG")) {
        std::cout << "Error loading space texture!" << std::endl;
        return -1;
    }

    sf::Texture sunTexture;
    if (!sunTexture.loadFromFile("Sun.PNG")) {
        std::cout << "Error loading sun texture!" << std::endl;
        return -1;
    }

    // Create Space Background
    sf::Sprite spaceBackground;
    spaceBackground.setTexture(spaceTexture);
    spaceBackground.setScale(
        static_cast<float>(window.getSize().x) / spaceTexture.getSize().x,
        static_cast<float>(window.getSize().y) / spaceTexture.getSize().y
    );

    // Box2D World Setup
    b2Vec2 gravity(0.0f, 0.0f); // Disable default gravity
    b2World world(gravity);

    // Contact Listener
    MyContactListener contactListener;
    world.SetContactListener(&contactListener);

    // Sun Setup
    b2BodyDef sunDef;
    sunDef.position.Set(400.0f / SCALE, 300.0f / SCALE); // Sun at the center
    b2Body* sun = world.CreateBody(&sunDef);

    b2CircleShape sunShape;
    sunShape.m_radius = 70.0f / SCALE;

    b2FixtureDef sunFixtureDef;
    sunFixtureDef.shape = &sunShape;
    sun->CreateFixture(&sunFixtureDef);
    sun->GetUserData().pointer = reinterpret_cast<uintptr_t>("sun");

    // SFML Graphics for Sun
    sf::CircleShape sunStar(70.0f);
    sunStar.setTexture(&sunTexture);
    /* sunStar.setFillColor(sf::Color::Yellow);*/
    sunStar.setOrigin(70.0f, 70.0f);
    sunStar.setPosition(400.0f, 300.0f);
    

    // Dynamic planets and orbits
    std::vector<b2Body*> planets;
    std::vector<sf::CircleShape> planetShapes;
    std::vector<std::vector<sf::Vector2f>> orbits; // History of positions for each planet

    while (window.isOpen()) {
        sf::Event event;
        sf::Color PlanetColor(rand() % 255, rand() % 255, rand() % 255);
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                sf::Vector2i mousePos = sf::Mouse::getPosition(window);
                float radius = (std::rand() % 25) + 5;

                // Planet Body Setup
                b2BodyDef planetDef;
                planetDef.type = b2_dynamicBody;
                planetDef.position.Set(mousePos.x / SCALE, mousePos.y / SCALE);
                b2Body* newPlanet = world.CreateBody(&planetDef);

                //Planet shape
                b2CircleShape planetShape;
                planetShape.m_radius = radius / SCALE;

                //Planet fixture
                b2FixtureDef planetFixture;
                planetFixture.shape = &planetShape;
                planetFixture.density = 1.0f;
                newPlanet->CreateFixture(&planetFixture);

                // Calculate initial velocity
                b2Vec2 sunPosition = sun->GetPosition();
                b2Vec2 planetPosition = newPlanet->GetPosition();
                b2Vec2 direction = sunPosition - planetPosition;
                float distance = direction.Length();  //distance between 2 points
                if (distance > 0.0f) {
                    direction.Normalize();
                    b2Vec2 tangential(-direction.y, direction.x); // Perpendicular to direction
                    float orbitalSpeed = std::sqrt(G / distance); // Orbital speed
                    newPlanet->SetLinearVelocity(orbitalSpeed * tangential);
                }

                // SFML Circle Setup
                sf::CircleShape planetShapeSFML(radius);
                planetShapeSFML.setFillColor(PlanetColor /*sf::Color(rand() % 255, rand() % 255, rand() % 255)*/);
                planetShapeSFML.setOrigin(radius, radius);
                planetShapeSFML.setPosition(mousePos.x, mousePos.y);

                planets.push_back(newPlanet);
                planetShapes.push_back(planetShapeSFML);

                // Add empty orbit for the new planet
                orbits.push_back(std::vector<sf::Vector2f>());

            }
        }

        // Apply gravitational force and decay for all planets
        for (size_t i = 0; i < planets.size(); ++i) {
            b2Body* planet = planets[i];
            b2Vec2 planetPosition = planet->GetPosition();
            b2Vec2 sunPosition = sun->GetPosition();
            b2Vec2 direction = sunPosition - planetPosition;
            float distance = direction.Length();

            if (distance > 0.0f) {
                direction.Normalize();
                float forceMagnitude = (G * planet->GetMass()) / (distance * distance);
                b2Vec2 gravitationalForce = forceMagnitude * direction;
                planet->ApplyForceToCenter(gravitationalForce, true);

                // Orbital decay
                b2Vec2 decay = -planet->GetLinearVelocity();
                decay *= 0.002 * distance;
                planet->ApplyForceToCenter(decay, true);
            }

            // Update orbit history
            orbits[i].push_back(sf::Vector2f(planetPosition.x * SCALE, planetPosition.y * SCALE));
            if (orbits[i].size() > 70) { // Limit orbit size
                orbits[i].erase(orbits[i].begin());
            }
        }
        sunStar.rotate(-1);


        // Step the simulation
        world.Step(1.0f / 60.0f, 8, 3);

        for (b2Body* body : contactListener.bodiesToDestroy) {
            auto it = std::find(planets.begin(), planets.end(), body);
            if (it != planets.end()) {
                size_t index = std::distance(planets.begin(), it);
                planets.erase(it);
                planetShapes.erase(planetShapes.begin() + index);
                orbits.erase(orbits.begin() + index);

            }
            world.DestroyBody(body);
        }
        contactListener.bodiesToDestroy.clear();



        // Update SFML shapes based on body position

        for (size_t i = 0; i < planets.size(); ++i) {
            b2Vec2 position = planets[i]->GetPosition();
            planetShapes[i].setPosition(position.x * SCALE, position.y * SCALE);
        }

        // Render
        window.clear();
        window.draw(spaceBackground);
        window.draw(sunStar);

        // Draw orbits
        for (const auto& orbit : orbits) {
            sf::VertexArray orbitLine(sf::LineStrip, orbit.size());
            for (size_t j = 0; j < orbit.size(); ++j) {
                orbitLine[j].position = orbit[j];
                orbitLine[j].color = sf::Color(rand() % 255, rand() % 255, rand() % 255); //  color
            }
            window.draw(orbitLine);
        }


        // Draw planets
        for (const auto& shape : planetShapes) {
            window.draw(shape);
        }

        window.display();
    }

    return 0;
}
