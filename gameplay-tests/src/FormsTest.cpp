#include "FormsTest.h"

#if defined(ADD_TEST)
    ADD_TEST("Graphics", "Forms", FormsTest, 101);
#endif

const static unsigned int __formsCount = 5;

FormsTest::FormsTest()
    : _scene(NULL), _formNode(NULL), _formSelect(NULL), _activeForm(NULL)
{
    const char* formFiles[] = 
    {
        "res/common/ui/test.form",
        "res/common/ui/Scrolling.form",
        "res/common/ui/FlowLayout.form",
        "res/common/ui/VerticalLayout.form",
        "res/common/ui/ZOrder.form",
    };

    _formFiles.assign(formFiles, formFiles + __formsCount);
}

FormsTest::~FormsTest()
{
}

FormsTest* FormsTest::create()
{
    return new FormsTest();
}

void FormsTest::finalize()
{
    SAFE_RELEASE(_scene);
    SAFE_RELEASE(_formSelect);
    for (unsigned int i = 0; i < _forms.size(); i++)
    {
        SAFE_RELEASE(_forms[i]);
    }
}

void printProperties(Properties* properties, unsigned int tabCount)
{
    // Print the name and ID of the current namespace.
    const char* spacename = properties->getNamespace();
    const char* id = properties->getId();
    std::string tabs;
    for (unsigned int i = 0; i < tabCount; i++)
    {
        tabs.append("\t");
    }
    GP_WARN("\n%s%s %s\n%s{", tabs.c_str(), spacename, id, tabs.c_str());
 
    // Print all properties in this namespace.
    const char* name = properties->getNextProperty();
    const char* value = NULL;
    while (name != NULL)
    {
        value = properties->getString(name);
        GP_WARN("%s\t%s = %s", tabs.c_str(), name, value);
        name = properties->getNextProperty();
    }
 
    // Print the properties of every namespace within this one.
    Properties* space = properties->getNextNamespace();
    while (space != NULL)
    {
        printProperties(space, tabCount+1);
        space = properties->getNextNamespace();
    }

    GP_WARN("%s}", tabs.c_str());
}

void FormsTest::initialize()
{
    setVsync(false);

    _formSelect = Form::create("res/common/ui/formSelect.form");
    
    RadioButton* form0Button = static_cast<RadioButton*>(_formSelect->getControl("form0"));
    form0Button->addListener(this, Control::Listener::CLICK);

    RadioButton* form1Button = static_cast<RadioButton*>(_formSelect->getControl("form1"));
    form1Button->addListener(this, Control::Listener::CLICK);

    RadioButton* form2Button = static_cast<RadioButton*>(_formSelect->getControl("form2"));
    form2Button->addListener(this, Control::Listener::CLICK);
    
    RadioButton* form3Button = static_cast<RadioButton*>(_formSelect->getControl("form3"));
    form3Button->addListener(this, Control::Listener::CLICK);

    RadioButton* form4Button = static_cast<RadioButton*>(_formSelect->getControl("form4"));
    form4Button->addListener(this, Control::Listener::CLICK);
    
    RadioButton* form5Button = static_cast<RadioButton*>(_formSelect->getControl("form5"));
    form5Button->addListener(this, Control::Listener::CLICK);
    
    for (unsigned int i = 0; i < _formFiles.size(); i++)
    {
        Form* form = Form::create(_formFiles[i]);
        form->disable();
        _forms.push_back(form);
    }
    _formIndex = 0;

    // Create a form programmatically.
    createTestForm(_forms[0]->getTheme()->getStyle("buttonStyle"));

    Button* button = static_cast<Button*>(_forms[0]->getControl("testButton"));
    button->addListener(this, Control::Listener::CLICK);

    // Create a scene with a camera node.
    Camera* camera = Camera::createPerspective(45.0f, (float)getWidth() / (float)getHeight(), 0.25f, 100.0f);
    _scene = Scene::create();
    Node* cameraNode = _scene->addNode("Camera");
    cameraNode->setCamera(camera);
    _scene->setActiveCamera(camera);
    SAFE_RELEASE(camera);
    _formNode = _scene->addNode("Form");
    
    formChanged();
}

void FormsTest::formChanged()
{
    if (_activeForm)
        _activeForm->disable();
    _activeForm = _forms[_formIndex];
    _activeForm->enable();

    // Add the form to a node to allow it to be placed in 3D space.
    const Rectangle& bounds = _activeForm->getBounds();
    float scale;
    if (bounds.width >= bounds.height)
    {
        scale = 1.0f / bounds.width;
    }
    else
    {
        scale = 1.0f / bounds.height;
    }

    _formNode->setScale(scale, scale, 1.0f);
    _formNode->setTranslation(-0.5f, -0.5f, -1.5f);
    _formNode->setForm(_activeForm);
}

void FormsTest::createTestForm(Theme::Style* style)
{
    Form* form = Form::create("testForm", style);
    form->setSize(600, 600);

    Label* label = Label::create("testLabel", style);
    label->setPosition(0, 10);
    label->setSize(200, 200);
    label->setText("This is a label.");
    form->addControl(label);
    label->release();

    Button* button = Button::create("testButton", style);
    button->setPosition(0, 210);
    button->setSize(200, 200);
    button->setText("This is a button.");
    form->addControl(button);
    button->release();

    form->disable();
    _forms.push_back(form);
}

void FormsTest::update(float elapsedTime)
{
    if (_formSelect)
    {
        _formSelect->update(elapsedTime);
    }

    if (_activeForm)
    {
        _activeForm->update(elapsedTime);
    }
}

void FormsTest::render(float elapsedTime)
{
    // Clear the screen.
    clear(CLEAR_COLOR_DEPTH, Vector4(0, 0, 0, 1), 1.0f, 0);

    // Draw the forms.
    if (_formSelect)
    {
        _formSelect->draw();
    }

    if (_activeForm)
    {
        _activeForm->draw();
    }
}

void FormsTest::touchEvent(Touch::TouchEvent evt, int x, int y, unsigned int contactIndex)
{
    if (_formNode)
    {
        switch (evt)
        {
        case Touch::TOUCH_PRESS:
            {
                _touched = true;
                _touchX = x;
            }
            break;
        case Touch::TOUCH_RELEASE:
            {
                _touched = false;
                _touchX = 0;
            }
            break;
        case Touch::TOUCH_MOVE:
            {
                int deltaX = x - _touchX;
                _touchX = x;
                _formNode->rotateY(MATH_DEG_TO_RAD(deltaX * 0.5f));
            }
            break;
        default:
            break;
        };
    }
}

void FormsTest::keyEvent(Keyboard::KeyEvent keyEvent, int key)
{
    if (_formNode)
    {
        switch(keyEvent)
        {
        case Keyboard::KEY_PRESS:
            switch (key)
            {
            case Keyboard::KEY_LEFT_ARROW:
                _formNode->translateX(-0.1f);
                break;
            case Keyboard::KEY_RIGHT_ARROW:
                _formNode->translateX(0.1f);
                break;
            case Keyboard::KEY_UP_ARROW:
                _formNode->translateY(0.1f);
                break;
            case Keyboard::KEY_DOWN_ARROW:
                _formNode->translateY(-0.1f);
                break;
            case Keyboard::KEY_PLUS:
                _formNode->translateZ(0.1f);
                break;
            case Keyboard::KEY_MINUS:
                _formNode->translateZ(-0.1f);
                break;
            }
            break;
        }
    }
}

void FormsTest::controlEvent(Control* control, EventType evt)
{
    if (strcmp("form0", control->getId()) == 0)
    {
        _formIndex = 0;
        formChanged();
    }
    else if (strcmp("form1", control->getId()) == 0)
    {
        _formIndex = 1;
        formChanged();
    }
    else if (strcmp("form2", control->getId()) == 0)
    {
        _formIndex = 2;
        formChanged();
    }
    else if (strcmp("form3", control->getId()) == 0)
    {
        _formIndex = 3;
        formChanged();
    }
    else if (strcmp("form4", control->getId()) == 0)
    {
        _formIndex = 4;
        formChanged();
    }
    else if (strcmp("form5", control->getId()) == 0)
    {
        _formIndex = 5;
        formChanged();
    }
}
