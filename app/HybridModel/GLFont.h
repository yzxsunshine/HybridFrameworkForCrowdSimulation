#ifndef	__GLFONT_H__
#define	__GLFONT_H__
#include <windows.h>
#include <GL/glut.h>
/** ���������� */
class GLFont															
{
public:
    /** ���캯������������ */
	GLFont();
	~GLFont();
    ///��Ա����
	bool InitFont();  /**< ��ʼ������ */
	void PrintText(const char *string, float x, float y, unsigned char* color); /**< ��(x,y)�����string���� */
	
protected:
	HFONT m_hFont;  /**< ������ */
		
};
#endif	// __GLFONT_H__