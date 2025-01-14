#include <iostream>
#include "pathing.h"
#include "utils.h"

// int main() {
// 	Vec2 one_one(1.f, 1.f);
// 	Vec2 zero_one(0.f, 1.f);
// 	Vec2 one_zero(1.f, 0.f);

// 	std::cout << "Distance: " << Vec2::distance(zero_one, one_zero) << std::endl;

// 	// std::cout << "len = " << one_one.norm() << std::endl;
// 	// Vec2 normalized = one_one.normalize();
// 	// std::cout << "normalized = {" << normalized.x << ", " << normalized.y << "}" << std::endl;
// }

#include <SFML/Audio.hpp>
#include <SFML/Graphics.hpp>

int main() {
	// Create the main window
	sf::RenderWindow window(sf::VideoMode(800, 600), "SFML window");

	// Load a sprite to display
	sf::Texture texture;
	if (!texture.loadFromFile("cute_image.jpg"))
		return EXIT_FAILURE;
	sf::Sprite sprite(texture);

	// Create a graphical text to display
	sf::Font font;
	if (!font.loadFromFile("arial.ttf"))
		return EXIT_FAILURE;
	sf::Text text("Hello SFML", font, 50);

	// Load a music to play
	sf::Music music;
	if (!music.openFromFile("nice_music.ogg"))
		return EXIT_FAILURE;

	// Play the music
	music.play();

	// Start the game loop
	while (window.isOpen()) {
		// Process events
		sf::Event event;
		while (window.pollEvent(event)) {
			// Close window: exit
			if (event.type == sf::Event::Closed)
				window.close();
		}

		// Clear screen
		window.clear();

		// Draw the sprite
		window.draw(sprite);

		// Draw the string
		window.draw(text);

		// Update the window
		window.display();
	}

	return EXIT_SUCCESS;
}