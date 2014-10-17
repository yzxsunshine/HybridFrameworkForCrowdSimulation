#ifndef	__GLFONT_H__
#define	__GLFONT_H__
#include <windows.h>
#include <GL/glut.h>
/** 定义字体类 */
class GLFont															
{
public:
    /** 构造函数和析构函数 */
	GLFont();
	~GLFont();
    ///成员方法
	bool InitFont();  /**< 初始化字体 */
	void PrintText(const char *string, float x, float y, unsigned char* color); /**< 在(x,y)处输出string内容 */
	
protected:
	HFONT m_hFont;  /**< 字体句柄 */
		
};
#endif	// __GLFONT_H__