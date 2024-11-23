#include <SFML/Graphics.hpp>
#include <box2d/box2d.h>
#include <set>
#include <cmath>
#include <cstring>

// Constants
const float SCALE = 30.0f;              // Pixels per Box2D unit
const float G = 200.0f;                // Gravitational constant
const float DECAY_MULTIPLIER = 0.02f; // Decay adjustment factor

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
            b2Body* target;
            if (userDataA && strcmp((char*)userDataA, "sun") == 0)
            {
                target = bodyB;
            } 
            else 
            {
                target =bodyA;
            }
            bodiesToDestroy.insert(target);
        }
    }
};

int main() {
    // SFML Setup
    sf::RenderWindow window(sf::VideoMode(800, 600), "Sun-Based Gravity with Orbit Decay");
    window.setFramerateLimit(60);

    // Box2D World Setup
    b2Vec2 gravity(0.0f, 0.0f); // Disable default gravity
    b2World world(gravity);

    // Contact Listener
    MyContactListener contactListener;
    world.SetContactListener(&contactListener);

    // Sun Setup
    b2BodyDef sunDef;
    sunDef.position.Set(400.0f / SCALE, 300.0f / SCALE); // Sun at the center -------------------------?????????????????????????????
    b2Body* sun = world.CreateBody(&sunDef);

    //Sun shape
    b2CircleShape sunShape;
    sunShape.m_radius = 30.0f / SCALE;

    //FIXTURES OF THE SUN 
    b2FixtureDef sunFixtureDef;
    sunFixtureDef.shape = &sunShape;
    sun->CreateFixture(&sunFixtureDef);
    sun->GetUserData().pointer = reinterpret_cast<uintptr_t>("sun");

    // Planet Setup
    b2BodyDef planetDef;
    planetDef.type = b2_dynamicBody;
    planetDef.position.Set(600.0f / SCALE, 300.0f / SCALE); // Start to the right of the sun
    b2Body* planet = world.CreateBody(&planetDef);

    //Planet shape
    b2CircleShape planetShape;
    planetShape.m_radius = 15.0f / SCALE;

    //Fixture of the planet
    b2FixtureDef planetFixtureDef;
    planetFixtureDef.shape = &planetShape;
    planetFixtureDef.density = 1.0f;
    //planetFixtureDef.friction = 0.0f;    // No friction
    //planetFixtureDef.restitution = 0.0f; // No bounce
    planet->CreateFixture(&planetFixtureDef);

    // Add initial tangential velocity for orbit //-----------try & error
    float initialSpeed = 5.0f; // Adjust for a stable orbit
    b2Vec2 tangentialVelocity(0.0f, -initialSpeed); // Tangential velocity perpendicular to the sun
    planet->SetLinearVelocity(tangentialVelocity);

    // SFML Graphics for Sun
    sf::CircleShape sunStar(30.0f); // Radius in pixels
    sunStar.setFillColor(sf::Color::Yellow);
    sunStar.setOrigin(30.0f, 30.0f); // Center the origin
    sunStar.setPosition(400.0f, 300.0f); // Center of the screen

    // SFML Graphics for Planet
    sf::CircleShape planetShapeSFML(15.0f); // Radius in pixels
    planetShapeSFML.setFillColor(sf::Color::Blue);
    planetShapeSFML.setOrigin(15.0f, 15.0f);

    // Main Loop
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        // Apply gravitational force between sun and planet
        b2Vec2 planetPosition = planet->GetPosition();
        b2Vec2 sunPosition = sun->GetPosition();

        b2Vec2 direction = sunPosition - planetPosition; // Vector from planet to sun
        float distance = direction.Length(); //distance between 2 points

        if (distance > 0.0f) {
            direction.Normalize(); // Unit vector
            float forceMagnitude = (G * planet->GetMass()) / (distance * distance); //newton
            b2Vec2 gravitationalForce = forceMagnitude * direction;
            planet->ApplyForceToCenter(gravitationalForce, true);

            // Simulate orbital decay (adjust speed to move closer over time)
            b2Vec2 decay = -planet->GetLinearVelocity();
            decay *= DECAY_MULTIPLIER * distance; // Decay proportional to distance
            planet->ApplyForceToCenter(decay, true);
        }

        // Step the simulation
        float timeStep = 1.0f / 60.0f;
        /*int velocityIterations = 8;
        int positionIterations = 3;*/
        world.Step(timeStep, 0, 0);

        // Destroy bodies marked for removal
        for (b2Body* body : contactListener.bodiesToDestroy) {
            world.DestroyBody(body);
        }
        contactListener.bodiesToDestroy.clear();

        // Update SFML shapes dynamically
        if (planet->IsEnabled()) {
            planetShapeSFML.setPosition(planet->GetPosition().x * SCALE, planet->GetPosition().y * SCALE);
        }

        // Render
        window.clear();
        window.draw(sunStar);
        if (planet->IsEnabled()) {
            window.draw(planetShapeSFML);
        }
        window.display();
    }

    return 0;
}
