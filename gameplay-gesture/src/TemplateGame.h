#ifndef TEMPLATEGAME_H_
#define TEMPLATEGAME_H_

#include "gameplay.h"
#include "LineDrawer.h"
#include "GestureTemplate.h"

using namespace gameplay;

/**
 * Main game class.
 */
class GestureRecognizer;
class TemplateGame: public Game
{
public:

    /**
     * Constructor.
     */
    TemplateGame();

    /**
     * @see Game::keyEvent
     */
	void keyEvent(Keyboard::KeyEvent evt, int key);
	
    /**
     * @see Game::touchEvent
     */
    void touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex);

protected:

    /**
     * @see Game::initialize
     */
    void initialize();

    /**
     * @see Game::finalize
     */
    void finalize();

    /**
     * @see Game::update
     */
    void update(float elapsedTime);

    /**
     * @see Game::render
     */
    void render(float elapsedTime);

	void drawSplash(void* param);
private:

    /**
     * Draws the scene each frame.
     */
    bool drawScene(Node* node);
	void drawFrameRate(Font* font, const Vector4& color, unsigned int x, unsigned int y, unsigned int fps);
	void addVertex(int x, int y);
	void recognize();
	GestureTemplate generateRandomTemplate(const GesturePoints& base, const std::string& id, int maxPoints);
	void generateTwoProgressiveRandomTemplates(std::vector<GestureTemplate> *outout);

    Scene* _scene;
	Node *_cameraParent, *_particleEmitterNode;
	Font *_font;
	LineDrawer *_lineDw;

	GesturePoints _vertices;

    double _lastVertexAdded;
	Matrix _worldViewProjectionMatrix;
	GestureRecognizer *_recognizer;
};

#endif
