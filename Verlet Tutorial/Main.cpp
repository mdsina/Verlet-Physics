#include "Main.h"
#include <time.h>


int GWidth  = 1024; //Разрешение экрана
int GHeight = 600;

int MouseX = 0; //Координаты мыши по-умолчанию
int MouseY = 0;
bool k = true;
Vertex* DragVertex = 0; //Перетаскиваемая мышью вершина

Physics World( 0.0f, 0.5f, 10); //Создаем "мир" с точность рассчетов =20;

void Render() {
	glClear( GL_COLOR_BUFFER_BIT ); //Очистка экрана

	World.Render(); //Отрисовка мира

	glFlush(); //Рендерить все в обязательном порядке(что бы без глюков)
}

void DispatchKeyboard( unsigned char Key, int X, int Y ) { 
	if( Key == 27 ) exit( 0 );
	
}

void DispatchTimer( int ID ) { //обновляет и рендерит все с частотой в 60 кадров\с (или меньше, если метод обновления занимает больше времени)
	World.Update();

	Render();

	if( DragVertex != 0 ) 
		DragVertex->Position = Vec2( (float)MouseX, (float)MouseY ); //Устанавливаем позицию точки, которую будем перетаскивать, к координатам мыши

	glutTimerFunc( (int)( 1000.0f/60.0f ), DispatchTimer, 0 ); //Вызываем таймер через каждые 16 мс
}

void DispatchMouseMotion( int X, int Y ) { //Записываем координаты мыши
	MouseX = X;
	MouseY = Y;
}

void DispatchMouseClick( int Button, int State, int X, int Y ) {

	if( Button == GLUT_LEFT_BUTTON ) {
		if( State == GLUT_DOWN ) { //Устанавливаем перетаскиваемой вершине координаты объекта ближайшей вершины, если таковая найдена.
			if( !DragVertex )
				DragVertex = World.FindVertex( X, Y );
			else
				DragVertex = 0;
		}
	} else if( Button == GLUT_RIGHT_BUTTON && State == GLUT_DOWN ) {
		int k = rand ( ) % 4;
		switch (k)
		{
		case 0: ( new PhysicsBody() )->CreateSoftCircle( X, Y, 30+rand()%40, 4+rand()%10 );
			break;
		case 1: ( new PhysicsBody() )->CreateBox( X, Y, 20+rand()%40, 20+rand()%40 );
			break;
		case 2: ( new PhysicsBody() )->CreateCircle( X, Y, 10+rand()%40, 4+rand()%10 );
			break;
		case 3: ( new PhysicsBody() )->CreateTriangle( X, Y);
		}
		 
		
	}
}

void InitPhysics() {
	
	for( int Y = GHeight-50; Y > 100; Y -= 50)
		( new PhysicsBody() )->CreateBox( GWidth-50, Y, 50, 50 );
	for( int Y = GHeight-50; Y > 100; Y -= 50)
		( new PhysicsBody() )->CreateBox( GWidth-100, Y, 50, 50 );
	for( int Y = GHeight-50; Y > 100; Y -= 50)
		( new PhysicsBody() )->CreateBox( GWidth-150, Y, 50, 50 );
	for( int Y = GHeight-50; Y > 100; Y -= 50)
		( new PhysicsBody() )->CreateBox( GWidth-200, Y, 50, 50 );
	( new PhysicsBody() )->CreateBox( 100, 100, 100, 100 );
	( new PhysicsBody() )->CreateBox( 100, 100, 20, 179 );
	//( new PhysicsBody() )->CreateCircle( 20, 20, 100 );
	
}

void InitGL() {
	glMatrixMode( GL_PROJECTION ); //Устанавливаем проекционную и видовую матрицу
	glLoadIdentity();
	
	glOrtho( 0, GWidth, GHeight, 0, -1, 1 );
	
	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
}

int main( int argc, char **argv ) {
	glutInit( &argc, argv ); //Инициализация glut
	srand ( time ( NULL ) );
	glutInitDisplayMode( GLUT_DEPTH | GLUT_SINGLE | GLUT_RGBA );

	glutInitWindowPosition( 100, 100 );
	glutInitWindowSize    ( GWidth, GHeight );

	glutCreateWindow( "OLOLO" );

	glutDisplayFunc      ( Render );
	glutTimerFunc        ( (int)( 100.0f/60.0f ), DispatchTimer, 0 );
	glutPassiveMotionFunc( DispatchMouseMotion );
	glutMouseFunc        ( DispatchMouseClick );
	glutKeyboardFunc     ( DispatchKeyboard );

	InitGL(); //Инициализция openGL
	InitPhysics(); //Инциализация физики

	glutMainLoop(); //Основной цикл
	
	return 0;
}
