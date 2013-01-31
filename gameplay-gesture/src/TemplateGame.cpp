#include "TemplateGame.h"

// Declare our game instance
TemplateGame game;

TemplateGame::TemplateGame()
    : _scene(NULL)
{
}

void TemplateGame::initialize()
{
	setMultiTouch(true);

    _scene = Scene::create();
    Node* cameraNode = _scene->addNode("Camera");
    _cameraParent = _scene->addNode("CameraParent");
    _cameraParent->addChild(cameraNode);
    Camera* camera = Camera::createPerspective(45.0f, (float)getWidth() / (float)getHeight(), 0.25f, 1000.0f);
    cameraNode->setCamera(camera);
    cameraNode->setTranslation(0.0f, 0.0f, 40.0f);
    _scene->setActiveCamera(camera);
    SAFE_RELEASE(camera);

    // Create a font for drawing the framerate.
    _font = Font::create("res/arial.gpb");

	// Create particle emmiter
	ParticleEmitter* emitter = ParticleEmitter::create("res/fire.particle");

    _particleEmitterNode = _scene->addNode("Particle Emitter");
    _particleEmitterNode->setTranslation(0.0f, 0.0f, 0.0f);

    // Set the new emitter on the node.
    _particleEmitterNode->setParticleEmitter(emitter);
	emitter->start();

	// initialize LineDrawer
	_lineDw = new LineDrawer();
	Matrix::createOrthographic(getWidth(), getHeight(), -1.0f, 1.0f, &_worldViewProjectionMatrix);

	// Create gesture recognizer
	_recognizer = new GestureRecognizer();
	// Generate templates
	std::vector<GestureTemplate> templates;
	generateTwoProgressiveRandomTemplates(&templates);
	_recognizer->setTemplates(templates);

    // Reset camera view and zoom.
    _scene->getActiveCamera()->getNode()->setTranslation(0.0f, 0.0f, 40.0f);
    _cameraParent->setIdentity();

	
}

void TemplateGame::finalize()
{
    SAFE_RELEASE(_scene);
	delete _lineDw;
}

void TemplateGame::update(float elapsedTime)
{
    // Update particles.
    _particleEmitterNode->getParticleEmitter()->update(elapsedTime);
}

void TemplateGame::render(float elapsedTime)
{
    // Clear the color and depth buffers
    clear(CLEAR_COLOR_DEPTH, Vector4::zero(), 1.0f, 0);


	_lineDw->begin(_worldViewProjectionMatrix);
	if(_vertices.size() > 1) {
		for(int i = 1; i < _vertices.size(); ++i) {
			_lineDw->drawLine(Vector3(_vertices[i - 1].x, _vertices[i - 1].y, 0.0f),
				Vector3(_vertices[i].x, _vertices[i].y, 0.0f), 
				Vector3(1.0f, 0.0f, 0.0f));
		}
	}

	_lineDw->end();
}

void TemplateGame::recognize()
{
	if(_vertices.size() < 2)
		return;
	std::list<GestureResult> results = _recognizer->recognize(_vertices);
	results;
}

 GestureTemplate TemplateGame::generateRandomTemplate(const GesturePoints& base, const std::string& id, int maxPoints) 
 {
		GesturePoints points;
		if (base.size() > 0) {
			points.insert(points.end(), base.begin(), base.end());
		}
		for (int i = 0; i < maxPoints; i++) {
			int x = -500 + (int)(MATH_RANDOM_0_1() * 1000.0);
			int y = -500 + (int)(MATH_RANDOM_0_1() * 1000.0);
			points.push_back(Vector2(x, y));
		}
		return GestureTemplate(id, points);
}

void TemplateGame::generateTwoProgressiveRandomTemplates(std::vector<GestureTemplate> *output) 
{
	for (int i = 1; i < 9; i++) {
		if (i < 3) {
			output->push_back(generateRandomTemplate(GesturePoints(), "Random ", 2));
		} else {
			output->push_back(generateRandomTemplate(output->at(i - 3)._points, "Random ", 1));
		}
	}
}

bool TemplateGame::drawScene(Node* node)
{
    // If the node visited contains a model, draw it
    ParticleEmitter* emitter = node->getParticleEmitter();
    if (emitter)
    {
        emitter->draw();
    }

	    // Draw the framerate and number of live particles.
    drawFrameRate(_font, Vector4(0, 0.5f, 1, 1), 170, 10, getFrameRate());
    return true;
}

void TemplateGame::keyEvent(Keyboard::KeyEvent evt, int key)
{
    if (evt == Keyboard::KEY_PRESS)
    {
        switch (key)
        {
        case Keyboard::KEY_ESCAPE:
            exit();
            break;
		case Keyboard::KEY_BACKSPACE:
			_vertices.pop_back();
			recognize();
			break;
		}
    }
}

void TemplateGame::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
    switch (evt) {
		case Touch::TOUCH_PRESS:
			addVertex( x - (getWidth() >> 1), (getHeight() >> 1) - y);
		break;
		case Touch::TOUCH_RELEASE:
			recognize();
		break;
		case Touch::TOUCH_MOVE:
			if (Game::getInstance()->getAbsoluteTime() - _lastVertexAdded > 20) {
				addVertex( x - (getWidth() >>1), (getHeight() >> 1) - y);
				recognize();
			}
		break;
	};
}

void TemplateGame::drawSplash(void* param)
{
    clear(CLEAR_COLOR_DEPTH, Vector4(0, 0, 0, 1), 1.0f, 0);
    SpriteBatch* batch = SpriteBatch::create("res/logo_powered_white.png");
    batch->start();
    batch->draw(this->getWidth() * 0.5f, this->getHeight() * 0.5f, 0.0f, 512.0f, 512.0f, 0.0f, 1.0f, 1.0f, 0.0f, Vector4::one(), true);
    batch->finish();
    SAFE_DELETE(batch);
}

void TemplateGame::drawFrameRate(Font* font, const Vector4& color, unsigned int x, unsigned int y, unsigned int fps)
{
    char buffer[30];
    sprintf(buffer, "FPS: %u\n", fps);
    font->start();
    font->drawText(buffer, x, y, color, 28);
    font->finish();
}

void TemplateGame::addVertex(int x, int y)
{
    Vector2 p(x, y);
    _vertices.push_back(p);

    
	_lastVertexAdded = Game::getInstance()->getAbsoluteTime();
}