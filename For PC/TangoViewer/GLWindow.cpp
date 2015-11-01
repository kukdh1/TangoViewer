#include "GLWindow.h"

namespace kukdh1
{
	GLWindow::GLWindow(HWND hParent, HINSTANCE hInstance, LPRECT rtWindow, GLDrawFunction gdfDrawFunction)
	{
		wsprintf(lpszGLClass, L"OpenGL Window");

		RegisterOpenGL(hInstance);

		CopyRect(&this->rtWindow, rtWindow);

		hWindow = CreateWindow(lpszGLClass, lpszGLClass, WS_CHILD | WS_VISIBLE, rtWindow->left, rtWindow->top, rtWindow->right - rtWindow->left, rtWindow->bottom - rtWindow->top, hParent, NULL, hInstance, NULL);
		SendMessage(hWindow, WM_REGISTER_THIS, NULL, (LPARAM)this);

		gdfDraw = gdfDrawFunction;
	}

	GLWindow::~GLWindow()
	{
		//Nothing To Do
	}

	BOOL GLWindow::SetupPixelFormat(HDC hDC)
	{
		PIXELFORMATDESCRIPTOR pfd = { sizeof(PIXELFORMATDESCRIPTOR), 1, PFD_SUPPORT_OPENGL | PFD_DRAW_TO_WINDOW | PFD_DOUBLEBUFFER, PFD_TYPE_RGBA, 32, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 24, 8, 0, PFD_MAIN_PLANE, 0, 0, 0, 0 };
		int pixelFormat;

		pixelFormat = ChoosePixelFormat(hDC, &pfd);
		if (pixelFormat == 0)
			return FALSE;

		if (SetPixelFormat(hDC, pixelFormat, &pfd) != TRUE)
			return FALSE;

		return TRUE;
	}

	BOOL GLWindow::RegisterOpenGL(HINSTANCE hInstance)
	{
		WNDCLASS wndclass;

		wndclass.cbClsExtra = 0;
		wndclass.cbWndExtra = 0;
		wndclass.hbrBackground = (HBRUSH)GetSysColorBrush(COLOR_BTNFACE);
		wndclass.hCursor = LoadCursor(NULL, IDC_ARROW);
		wndclass.hIcon = LoadIcon(NULL, IDI_APPLICATION);
		wndclass.hInstance = hInstance;
		wndclass.lpfnWndProc = (WNDPROC)OpenGLProc;
		wndclass.lpszMenuName = NULL;
		wndclass.lpszClassName = lpszGLClass;
		wndclass.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;

		return RegisterClass(&wndclass);
	}

	void * GLWindow::operator new(size_t i)
	{
		return _aligned_malloc(i, 16);
	}

	void GLWindow::operator delete(void *ptr)
	{
		_aligned_free(ptr);
	}

	void GLWindow::Refresh()
	{
		InvalidateRect(hWindow, NULL, FALSE);
	}

	void GLWindow::OnWheelMessage(HWND hParent, WPARAM wParam, LPARAM lParam)
	{
		WINDOWINFO wiThis;
		kukdh1::MOUSE_POINT mpMouse, mpLTPoint;

		wiThis.cbSize = sizeof(WINDOWINFO);

		GetWindowInfo(hParent, &wiThis);

		mpMouse.x = GET_X_LPARAM(lParam);
		mpMouse.y = GET_Y_LPARAM(lParam);
		mpLTPoint.x = wiThis.rcClient.left;
		mpLTPoint.y = wiThis.rcClient.top;
		mpMouse -= mpLTPoint;

		if (PtInRect(&rtWindow, mpMouse))
			SendMessage(hWindow, WM_MOUSEWHEEL_PASS, wParam, lParam);
	}

	LRESULT CALLBACK GLWindow::OpenGLProc(HWND hWnd, UINT iMessage, WPARAM wParam, LPARAM lParam)
	{
		GLWindow * pglwThis;
		MOUSE_POINT npt, diff;
		HDC hdc;
		PAINTSTRUCT ps;
		glm::mat4 m4Temp;

		switch (iMessage)
		{
			case WM_REGISTER_THIS:
				if (lParam)
				{
					pglwThis = (GLWindow *)lParam;
					SetClassObject(hWnd, pglwThis);

					hdc = GetDC(hWnd);
					pglwThis->SetupPixelFormat(hdc);
					pglwThis->hGLRC = wglCreateContext(hdc);
					wglMakeCurrent(hdc, pglwThis->hGLRC);
					ReleaseDC(hWnd, hdc);

					glViewport(0, 0, pglwThis->rtWindow.right - pglwThis->rtWindow.left, pglwThis->rtWindow.bottom - pglwThis->rtWindow.top);

					pglwThis->m4PivotTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -10.f));

					glMatrixMode(GL_PROJECTION);
					glLoadIdentity();
					gluPerspective(55, (pglwThis->rtWindow.right - pglwThis->rtWindow.left) / (pglwThis->rtWindow.bottom - pglwThis->rtWindow.top), 1.5, 100.0);

					glMatrixMode(GL_MODELVIEW);
					glLoadIdentity();

					glMultMatrixf(glm::value_ptr(pglwThis->m4PivotTranslation));
					glMultMatrixf(glm::value_ptr(pglwThis->m4CurrentTransform));

					glEnable(GL_DEPTH_TEST);

					glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
					glClearColor(1.0, 1.0, 1.0, 1.0);
				}

				return 0;
			case WM_MOUSEWHEEL_PASS:
				pglwThis = GetClassObject(hWnd);
				
				if (pglwThis)
				{
					m4Temp = glm::translate(glm::mat4(1.f), glm::vec3(0.f, 0.f, GL_ZOOM_RATIO * GET_WHEEL_DELTA_WPARAM(wParam)));
					pglwThis->m4CurrentTransform = m4Temp * pglwThis->m4CurrentTransform;
					InvalidateRect(hWnd, NULL, FALSE);
				}

				return 0;
			case WM_LBUTTONDOWN:
				pglwThis = GetClassObject(hWnd);
				
				if (pglwThis)
				{
					POINTSTOPOINT(pglwThis->mpLastMoustPoint, MAKEPOINTS(lParam));

					SetCapture(hWnd);

					pglwThis->bLButtonDown = TRUE;
				}

				return 0;
			case WM_LBUTTONUP:
				pglwThis = GetClassObject(hWnd);

				if (pglwThis)
				{
					ReleaseCapture();
					pglwThis->bLButtonDown = FALSE;
				}

				return 0;
			case WM_RBUTTONDOWN:
				pglwThis = GetClassObject(hWnd);

				if (pglwThis)
				{
					POINTSTOPOINT(pglwThis->mpLastMoustPoint, MAKEPOINTS(lParam));

					SetCapture(hWnd);

					pglwThis->bRButtonDown = TRUE;
				}

				return 0;
			case WM_RBUTTONUP:
				pglwThis = GetClassObject(hWnd);

				if (pglwThis)
				{
					ReleaseCapture();
					pglwThis->bRButtonDown = FALSE;
				}

				return 0;
			case WM_MOUSEMOVE:
				pglwThis = GetClassObject(hWnd);

				if (pglwThis)
				{
					POINTSTOPOINT(npt, MAKEPOINTS(lParam));

					if (pglwThis->bLButtonDown || pglwThis->bRButtonDown)
					{
						diff = npt - pglwThis->mpLastMoustPoint;

						if (pglwThis->bLButtonDown)
							m4Temp = glm::translate(glm::mat4(1.f), glm::vec3(GL_MOVE_RATIO * diff.x, -GL_MOVE_RATIO * diff.y, 0.f));
						else if (pglwThis->bRButtonDown)
						{
							m4Temp = glm::rotate(glm::mat4(1.f), GL_ROT_RATIO * diff.y, glm::vec3(1.f, 0.f, 0.f));
							m4Temp = glm::rotate(m4Temp, GL_ROT_RATIO * diff.x, glm::vec3(0.f, 1.f, 0.f));
						}

						pglwThis->m4CurrentTransform = m4Temp * pglwThis->m4CurrentTransform;

						InvalidateRect(hWnd, NULL, FALSE);

						pglwThis->mpLastMoustPoint = npt;
					}
				}

				return 0;
			case WM_PAINT:
				pglwThis = GetClassObject(hWnd);

				if (pglwThis)
				{
					hdc = BeginPaint(hWnd, &ps);

					glMatrixMode(GL_MODELVIEW);
					glLoadIdentity();

					glMultMatrixf(glm::value_ptr(pglwThis->m4PivotTranslation));
					glMultMatrixf(glm::value_ptr(pglwThis->m4CurrentTransform));

					glEnable(GL_DEPTH_TEST);

					glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
					glClearColor(1.0, 1.0, 1.0, 1.0);

					// Draw XY-Plane Grid
					glBegin(GL_LINES);

					glColor3f(0.0, 0.0, 0.0);
					for (int i = -GL_GRID_SIZE; i <= GL_GRID_SIZE; i++)
					{
						if (i == 0)
						{
							glColor3f(1.f, 0.f, 0.f);
							glVertex3f(0.f, 0.f, 0.f);
							glVertex3f(GL_GRID_SIZE, 0.f, 0.f);

							glColor3f(0.f, 1.f, 0.f);
							glVertex3f(0.f, 0.f, 0.f);
							glVertex3f(0.f, GL_GRID_SIZE, 0.f);

							glColor3f(0.f, 0.f, 0.f);
							glVertex3f(0.f, 0.f, 0.f);
							glVertex3f(-GL_GRID_SIZE, 0.f, 0.f);

							glVertex3f(0.f, 0.f, 0.f);
							glVertex3f(0.f, -GL_GRID_SIZE, 0.f);
						}
						else
						{
							glVertex3f((float)i, -GL_GRID_SIZE, 0.f);
							glVertex3f((float)i, GL_GRID_SIZE, 0.f);

							glVertex3f(-GL_GRID_SIZE, (float)i, 0.f);
							glVertex3f(GL_GRID_SIZE, (float)i, 0.f);
						}
					}

					glEnd();

					// Draw Z-Axis
					glBegin(GL_LINES);

					glColor3f(0.f, 0.f, 1.f);
					glVertex3f(0.f, 0.f, 0.f);
					glVertex3f(0.f, 0.f, 5.f);

					glEnd();

					// Draw Objects
					m4Temp = glm::rotate(glm::mat4(1.0f), PI / 2, glm::vec3(1.0f, 0.0f, 0.0f));

					glMultMatrixf(glm::value_ptr(m4Temp));

					pglwThis->gdfDraw();

					SwapBuffers(hdc);

					EndPaint(hWnd, &ps);
				}

				return 0;
			case WM_DESTROY:
				pglwThis = GetClassObject(hWnd);

				if (pglwThis)
				{
					if (pglwThis->hGLRC)
					{
						wglMakeCurrent(NULL, NULL);
						wglDeleteContext(pglwThis->hGLRC);
					}
				}

				return 0;
		}

		return DefWindowProc(hWnd, iMessage, wParam, lParam);
	}

	GLWindow * GLWindow::GetClassObject(HWND hWnd)
	{
		return (GLWindow *)GetWindowLongPtr(hWnd, GWLP_USERDATA);
	}

	GLWindow * GLWindow::SetClassObject(HWND hWnd, GLWindow *pglwNew)
	{
		return (GLWindow *)SetWindowLongPtr(hWnd, GWLP_USERDATA, (LONG_PTR)pglwNew);
	}
}