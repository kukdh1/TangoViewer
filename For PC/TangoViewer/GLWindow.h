#pragma once

#ifndef _GL_WINDOW_H_
#define _GL_WINDOW_H_

#include <Windows.h>
#include <windowsx.h>
#include <gl/GL.h>
#include <gl/GLU.h>
#include "glm/mat4x4.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtc/matrix_transform.hpp"

#pragma comment(lib, "opengl32")
#pragma comment(lib, "glu32")

#define WM_REGISTER_THIS		WM_APP
#define	WM_MOUSEWHEEL_PASS		WM_APP + 1

#define GL_GRID_SIZE	50
#define GL_ROT_RATIO	0.01f
#define GL_MOVE_RATIO	0.01f
#define GL_ZOOM_RATIO	0.01f

namespace kukdh1
{
	/* MOUSE_POINT struct
	 *  This structure implement POINT struct
	 *  Add operator overloading functions
	 */
	struct MOUSE_POINT : POINT {
		MOUSE_POINT & operator=(const MOUSE_POINT & rhs)
		{
			if (this != &rhs)
			{
				x = rhs.x;
				y = rhs.y;
			}

			return *this;
		}

		MOUSE_POINT & operator+=(const MOUSE_POINT & rhs)
		{
			x += rhs.x;
			y += rhs.y;

			return *this;
		}

		MOUSE_POINT & operator-=(const MOUSE_POINT & rhs)
		{
			x -= rhs.x;
			y -= rhs.y;

			return *this;
		}

		const MOUSE_POINT operator-()
		{
			x = -x;
			y = -y;

			return *this;
		}

		const MOUSE_POINT operator+(const MOUSE_POINT &rhs)
		{
			return MOUSE_POINT(*this).operator+=(rhs);
		}

		const MOUSE_POINT operator-(const MOUSE_POINT &rhs)
		{
			return MOUSE_POINT(*this).operator-=(rhs);
		}
	};

	/* GLDrawFunction Function Pointer
	 *  Add draw functions inside of this function
	 */
	typedef void (*GLDrawFunction)();

	/* GLWindow class
	 *  OpenGL Wrapper class
	 */
	class GLWindow
	{
		private:
			HWND hWindow;
			RECT rtWindow;
			HGLRC hGLRC;
			BOOL bLButtonDown;
			BOOL bRButtonDown;

			WCHAR lpszGLClass[32];

			MOUSE_POINT mpLastMoustPoint;

			GLDrawFunction gdfDraw;

			glm::mat4 m4CurrentTransform;
			glm::mat4 m4PivotTranslation;

			BOOL SetupPixelFormat(HDC hDC);
			BOOL RegisterOpenGL(HINSTANCE hInstance);

			static GLWindow * GetClassObject(HWND hWnd);
			static GLWindow * SetClassObject(HWND hWnd, GLWindow *pglwNew);

		public:
			GLWindow(HWND hParent, HINSTANCE hInstance, LPRECT rtWindow, GLDrawFunction gdfDrawFunction);
			~GLWindow();

			void OnWheelMessage(HWND hParent, WPARAM wParam, LPARAM lParam);
			void Refresh();

			static LRESULT CALLBACK OpenGLProc(HWND hWnd, UINT iMessage, WPARAM wParam, LPARAM lParam);
	};
}

#endif