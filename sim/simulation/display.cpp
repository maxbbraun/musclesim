#include "display.h"

int Display::window_width;
int Display::window_height;
int Display::window_left;
int Display::window_top;

double Display::camera_x;
double Display::camera_y;
double Display::camera_z;

int Display::last_x;
int Display::last_y;

double Display::phi;
double Display::theta;
double Display::r;

Robot *Display::robot;
Controller *Display::controller;
bool Display::interactive;
bool Display::pause;
double Display::global_alpha;

void Display::initialize(int argc, char **argv) {
  // window size and position:

  window_width = WINDOW_WIDTH;
  window_height = WINDOW_HEIGHT;
  window_left = WINDOW_LEFT;
  window_top = WINDOW_TOP;

  // trackball:

  camera_x = 0;
  camera_y = 0;
  camera_z = 0;

  last_x = 0;
  last_y = 0;

  phi = PHI_START;
  theta = THETA_START;
  r = R_START;

  // initialize GLUT:

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_STENCIL);
  glutInitWindowSize(window_width, window_height);
  glutInitWindowPosition(window_left, window_top);
  glutCreateWindow("Artificial Muscle Simulator");
  glutDisplayFunc(Display::display);
  glutIdleFunc(Display::idle);
  glutMouseFunc(Display::mouse);
  glutMotionFunc(Display::motion);
  glutSpecialFunc(Display::special);
  glutKeyboardFunc(Display::keyboard);
  glutReshapeFunc(Display::reshape);

  // initialize OpenGL:

  glClearColor(0.9, 0.9, 0.9, 1.0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_RESCALE_NORMAL);
  glEnable(GL_NORMALIZE);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  reshape(window_width, window_height);
  motion(0, 0);

  // print infos:

  cout << "Controls:" << endl
       << " p"
       << "\t\t"
       << "Toggle pause" << endl
       << " i"
       << "\t\t"
       << "Toggle interaction" << endl
       << " 0..n"
       << "\t\t"
       << "Select actuator by index" << endl
       << " ./,"
       << "\t\t"
       << "Select next/previous actuator" << endl
       << " +/-"
       << "\t\t"
       << "Increase/decrease air pressure for selected actuator" << endl
       << " r"
       << "\t\t"
       << "Reset air pressure for all actuators" << endl
       << " Mouse"
       << "\t\t"
       << "Rotate view (mouse button down)" << endl
       << " UP/DN"
       << "\t\t"
       << "Zoom in/out" << endl
       << endl;
}

void Display::mainLoop(Robot *robot, Controller *controller) {
  Display::robot = robot;
  Display::controller = controller;

  interactive = false;
  pause = false;

  global_alpha = 1.0;

  glutMainLoop();
}

void Display::display() {
  // reset display:

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  gluLookAt(camera_x + ROTATION_CENTER_X, camera_y + ROTATION_CENTER_Y,
            camera_z + ROTATION_CENTER_Z, ROTATION_CENTER_X, ROTATION_CENTER_Y,
            ROTATION_CENTER_Z, 0, theta > M_PI ? -1 : 1, 0);

  // prepare mask for selected actuator:

  if (interactive) {
    glEnable(GL_STENCIL_TEST);
    glClear(GL_STENCIL_BUFFER_BIT);
    glStencilFunc(GL_ALWAYS, 1, 1);
    glStencilOp(GL_REPLACE, GL_REPLACE, GL_REPLACE);

    Actuator::maskSelected();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glStencilFunc(GL_EQUAL, 1, 1);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

    glDisable(GL_STENCIL_TEST);
  }

  glEnable(GL_LIGHTING);

  GLfloat shininess[] = {20.0};
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
  GLfloat ambient[] = {0.3, 0.3, 0.3, global_alpha};
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);

  // draw actuators:

  Actuator::drawAll(global_alpha);

  // draw primitives:

  Primitive::drawAll(global_alpha);

  // draw target:

  Target::draw(global_alpha);

  /*// draw cage:

  glDisable(GL_LIGHTING);

  glPushMatrix();
          glColor3f(0.2, 0.2, 0.2);
          glutWireSphere(CAGE_RADIUS, CAGE_DIVISIONS, CAGE_DIVISIONS);
  glPopMatrix();

  // draw coordinate system:

  glBegin(GL_LINES);
          glColor3f(1, 0, 0);
          glVertex3d(0, 0, 0);
          glVertex3d(10, 0, 0);
          glColor3f(0, 1, 0);
          glVertex3d(0, 0, 0);
          glVertex3d(0, 10, 0);
          glColor3f(0, 0, 1);
          glVertex3d(0, 0, 0);
          glVertex3d(0, 0, 10);
  glEnd();

  glEnable(GL_LIGHTING);*/

  // draw plane:

  if (Simulation::hasPlane()) {
    GLfloat plane_diffuse[] = {0.2, 0.2, 0.2, global_alpha};
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, plane_diffuse);
    // GLfloat plane_specular[] = {0.0, 0.0, 0.0, global_alpha};
    // glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, plane_specular);
    GLfloat plane_ambient[] = {1.0, 1.0, 1.0, global_alpha};
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, plane_ambient);

    glBegin(GL_TRIANGLE_FAN);
    glNormal3d(0, 1, 0);
    for (double j = PLANE_DIVISIONS; j >= 0; j--) {
      glVertex3d(cos(2 * M_PI * j / PLANE_DIVISIONS) * CAGE_RADIUS, 0,
                 sin(2 * M_PI * j / PLANE_DIVISIONS) * CAGE_RADIUS);
    }
    glEnd();
  }

  // mask selected actuator:

  if (interactive) {
    glEnable(GL_STENCIL_TEST);

    glPushMatrix();
    glLoadIdentity();

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_LIGHTING);

    glColor4f(1.0, 0.0, 0.0, 0.4 * global_alpha);
    glBegin(GL_QUADS);
    glVertex2d(-1, -1);
    glVertex2d(1, -1);
    glVertex2d(1, 1);
    glVertex2d(-1, 1);
    glEnd();

    glPopMatrix();

    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glDisable(GL_STENCIL_TEST);
  }

  // flush:

  glutSwapBuffers();
}

void Display::idle() {
  if (!pause) {
    // control:

    if (!interactive) {
      double *state = robot->getState();
      double *action = controller->output(state);
      robot->step(action);
    }

    // simulation step:

    Actuator::simulateAll();
    Simulation::doStep();

    // cout << "t = " << Simulation::getTime() << endl;

    // display:

    glutPostRedisplay();
  }
}

void Display::mouse(int button, int state, int x, int y) {
  if (state == GLUT_DOWN) {
    last_x = x;
    last_y = y;

    glutPostRedisplay();
  }
}

void Display::motion(int x, int y) {
  double dphi = ROTATION_SPEED * (x - last_x) / window_width;
  double dtheta = ROTATION_SPEED * (last_y - y) / window_height;
  theta += dtheta;
  phi += dphi;

  if (theta > 2 * M_PI) theta -= 2 * M_PI;
  if (phi > 2 * M_PI) phi -= 2 * M_PI;
  if (theta < 0) theta += 2 * M_PI;
  if (phi < 0) phi += 2 * M_PI;

  camera_x = r * sin(theta) * cos(phi);
  camera_y = r * cos(theta);
  camera_z = r * sin(theta) * sin(phi);

  last_x = x;
  last_y = y;

  glutPostRedisplay();
}

void Display::special(int key, int x, int y) {
  switch (key) {
    case GLUT_KEY_UP:
      r -= ZOOM_SPEED;
      break;
    case GLUT_KEY_DOWN:
      r += ZOOM_SPEED;
      break;
  }

  camera_x = r * sin(theta) * cos(phi);
  camera_y = r * cos(theta);
  camera_z = r * sin(theta) * sin(phi);

  glutPostRedisplay();
}

void Display::keyboard(unsigned char key, int x, int y) {
  double previous;
  bool ok;
  int selected_index;
  switch (key) {
    case 'P':;
    case 'p':
      pause ^= true;
      break;
    case 'I':;
    case 'i':
      interactive ^= true;
      cout << "Interaction is " << (interactive ? "on" : "off") << endl;
      break;
    case 'T':;
    case 't':
      global_alpha = (global_alpha == 1.0 ? 0.5 : 1.0);
      break;
    case '+':
      if (interactive) {
        previous = Actuator::getSelectedActuator()->getAirPressure();
        Actuator::getSelectedActuator()->setAirPressure(
            fmax(0, previous + PRESSURE_STEP));
        cout << "Air pressure of actuator #"
             << Actuator::getSelectedActuator()->getIndex()
             << " set to: " << Actuator::getSelectedActuator()->getAirPressure()
             << endl;
      }
      break;
    case '-':
      if (interactive) {
        previous = Actuator::getSelectedActuator()->getAirPressure();
        Actuator::getSelectedActuator()->setAirPressure(
            fmax(0, previous - PRESSURE_STEP));
        cout << "Air pressure of actuator #"
             << Actuator::getSelectedActuator()->getIndex()
             << " set to: " << Actuator::getSelectedActuator()->getAirPressure()
             << endl;
      }
      break;
    case 'R':;
    case 'r':
      if (interactive) {
        Actuator::resetAll();
        cout << "Reset air pressure." << endl;
      }
      break;
    case 'K':;
    case 'k':
      cout << "Simulation time is: " << Simulation::getTime() << " seconds."
           << endl;
      break;
    case 'L':;
    case 'l':
      if (interactive) {
        cout << "Length of actuator #"
             << Actuator::getSelectedActuator()->getIndex()
             << " is: " << Actuator::getSelectedActuator()->getLength() << endl;
      }
      break;
    case 'S':;
    case 's': {
      double *state = robot->getState();
      cout << "State of robot is: " << state[0] << " " << state[1] << " "
           << state[2] << endl;
      break;
    }
    case '.':
      if (interactive) {
        selected_index = Actuator::getSelectedActuator()->getIndex();
        ok = Actuator::selectActuator(selected_index + 1);
        if (ok)
          cout << "Selected actuator #"
               << Actuator::getSelectedActuator()->getIndex() << endl;
      }
      break;
    case ',':
      if (interactive) {
        selected_index = Actuator::getSelectedActuator()->getIndex();
        ok = Actuator::selectActuator(selected_index - 1);
        if (ok)
          cout << "Selected actuator #"
               << Actuator::getSelectedActuator()->getIndex() << endl;
      }
      break;
    default:
      if (interactive) {
        bool ok = Actuator::selectActuator(key - '0');
        if (ok)
          cout << "Selected actuator #"
               << Actuator::getSelectedActuator()->getIndex() << endl;
      }
      break;
  }
  glutPostRedisplay();
}

void Display::reshape(int new_width, int new_height) {
  window_width = new_width;
  window_height = new_height;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45, (GLdouble)window_width / window_height, 1, 4000);
  glViewport(0, 0, window_width, window_height);
}
