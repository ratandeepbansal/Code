#include <stdio.h>
#include <math.h>
#include <GL/glut.H>

/* array for mesh vertices */
int numVertices;
float (*vertices)[3];

/* array for mesh triangles */
int numTriangles;
int (*triangles)[3];

/* vertex normals */
float (*vertexNormals)[3];

/* array for mesh vertices (using PN triangles)*/
int numVertices0;
float (*vertices0)[3];

/*array for mesh triangles (using PN triangles)*/
int numTriangles0;
int (*triangles0)[3];

/* vertex normals using PN triangles*/
float (*vertexNormals0)[3];

/* triangle normals */
float (*faceNormals)[3];


/* the last mouse coordinates */
int mouseX, mouseY;

/* view information */
float qw, qx, qy, qz;
float zoom;
float shiftX, shiftY;
bool edge;
int width, height;


/* adding newly formed vertices */
void addVertex(float *p){

  vertices0[numVertices0][0] = p[0];
  vertices0[numVertices0][1] = p[1];
  vertices0[numVertices0][2] = p[2];
  numVertices0++;
}


/* adding newly formed triangles*/
void addTriangle(int a, int b, int c){
  triangles0[numTriangles0][0] = a;
  triangles0[numTriangles0][1] = b;
  triangles0[numTriangles0][2] = c;
  numTriangles0++;
}


void computeVertexNormals(){
  //Set all vectors to zero
  for( int i=0; i<numVertices; i++){
    vertexNormals[i][0] = 0;
    vertexNormals[i][1] = 0;
    vertexNormals[i][2] = 0;
  }
  
  //Sum up face normals (area-weighted)
  for( int i=0; i<numTriangles; i++){
    int* t = triangles[i];
    float* a = vertices[t[0]];
    float* b = vertices[t[1]];
    float* c = vertices[t[2]];
    //cross product of "ab" and "bc"
    float nx = (b[1]-a[1])*(c[2]-a[2]) - (b[2]-a[2])*(c[1]-a[1]);
    float ny = (b[2]-a[2])*(c[0]-a[0]) - (b[0]-a[0])*(c[2]-a[2]);
    float nz = (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]);
    
    for(int j=0; j<3; j++){
      vertexNormals[t[j]][0] += nx;
      vertexNormals[t[j]][1] += ny;
      vertexNormals[t[j]][2] += nz;
    }
  }
  
  //Normalize the vectors
  for( int i=0; i<numVertices; i++){
    float* n = vertexNormals[i];
    float l = (float)sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
    if(l != 0){
      n[0] /= l;
      n[1] /= l;
      n[2] /= l;
    }
  }
}


	//calculating w
int w(float *pi,float *pj,float *ni){
	float nx=pj[0]-pi[0];
	float ny=pj[1]-pi[1];
	float nz=pj[2]-pi[2];
	int num= nx*ni[0]+ny*ni[1]+nz*ni[2];
	return num;
}

	//assigning the values
void assign(float *b000, float *a0, float *b0, float *n){
	for(int j=0;j<3;j++){
		b000[j]=(2*a0[j]+b0[j]-w(a0,b0,n)*n[j])/3;
	}
}


	//Using PN triangles
void usePN(){
  numVertices0=0;
  numTriangles0=0;
  for(int i=0;i<numTriangles;i++){
	int *t= triangles[i];
	//reading vertices
	float *a=vertices[t[0]];
	float *b=vertices[t[1]];
	float *c=vertices[t[2]];
	//reading normals
	float *a1=vertexNormals[t[0]];
	float *b1=vertexNormals[t[1]];
	float *c1=vertexNormals[t[2]];

	float *b300=a;
	float *b030=b;
	float *b003=c;

	//vertices inside the old triangle
	float b210[3], b120[3], b021[3],b012[3], b102[3], b201[3], b111[3];

	assign(b210,a,b,a1);
	assign(b120,b,a,b1);
	assign(b021,b,c,b1);
	assign(b012,c,b,c1);
	assign(b102,c,a,c1);
	assign(b201,a,c,a1);


	float E[3],V[3];
	for(int j=0;j<3;j++){
		E[j]=(b210[j]+b120[j]+b021[j]+b012[j]+b102[j]+b201[j])/6;
		V[j]=(a[j]+b[j]+c[j])/3;
		b111[j]=E[j]+(E[j]-V[j])/2;
	}
	//adding the new vertices to the new list
	addVertex(b300);
	addVertex(b210);
	addVertex(b120);
	addVertex(b030);
	addVertex(b201);
	addVertex(b111);
	addVertex(b021);
	addVertex(b102);
	addVertex(b012);
	addVertex(b003);

	//adding new triangles to the new list


	int num0=numVertices0-1;
	addTriangle(num0,num0-2,num0-1);
	addTriangle(num0-1,num0-4,num0-3);
	addTriangle(num0-2,num0-4,num0-1);
	addTriangle(num0-2,num0-5,num0-4);
	addTriangle(num0-3,num0-7,num0-6);
	addTriangle(num0-4,num0-7,num0-3);
	addTriangle(num0-4,num0-8,num0-7);
	addTriangle(num0-5,num0-8,num0-4);
	addTriangle(num0-5,num0-9,num0-8);
  }
}



void computeFaceNormals(){
  for(int i=0; i<numTriangles0; i++){
    int* t = triangles0[i];
    float* a = vertices0[t[0]];
    float* b = vertices0[t[1]];
    float* c = vertices0[t[2]];
    //cross product of "ab" and "ac"
    float nx = (b[1]-a[1])*(c[2]-a[2]) - (b[2]-a[2])*(c[1]-a[1]);
    float ny = (b[2]-a[2])*(c[0]-a[0]) - (b[0]-a[0])*(c[2]-a[2]);
    float nz = (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0]);
    float l = (float)sqrt(nx*nx + ny*ny + nz*nz);
	    faceNormals[i][0] = nx/l;
		faceNormals[i][1] = ny/l;
		faceNormals[i][2] = nz/l;
  }
}

/*
void smoothShading(){
  glPolygonMode(GL_FRONT, GL_FILL);
  glShadeModel(GL_SMOOTH);   //flat shading 
  glEnable(GL_LIGHTING);
  
  for(int i=0; i<numTriangles0; i++){
    int *t = triangles0[i];
    glBegin(GL_POLYGON);
      glNormal3fv(vertexNormals[t[0]]);
      glVertex3fv(vertices0[t[0]]);
      glNormal3fv(vertexNormals[t[1]]);
      glVertex3fv(vertices0[t[1]]);
      glNormal3fv(vertexNormals[t[2]]);
      glVertex3fv(vertices0[t[2]]);
    glEnd();
  }
}
*/


void flatShading(){
  glPolygonMode(GL_FRONT, GL_FILL);
  glShadeModel(GL_FLAT);   /* flat shading */
  glEnable(GL_LIGHTING);
  
  for(int i=0; i<numTriangles0; i++){
    int *t = triangles0[i];
    glBegin(GL_POLYGON);
      glNormal3fv(faceNormals[i]);
      glVertex3fv(vertices0[t[0]]);
      glVertex3fv(vertices0[t[1]]);
      glVertex3fv(vertices0[t[2]]);
    glEnd();
  }
}


void drawMesh(){
  glPolygonMode(GL_FRONT, GL_LINE);
  glDisable(GL_LIGHTING);
  
  for(int i=0; i<numTriangles0; i++){
    int *t = triangles0[i];
    glBegin(GL_POLYGON);
      glVertex3fv(vertices0[t[0]]);
      glVertex3fv(vertices0[t[1]]);
      glVertex3fv(vertices0[t[2]]);
    glEnd();
  }
}

void display(void){ 
  /* display callback, clear frame buffer and z buffer, 
   rotate cube and draw, swap buffers */ 
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
  glLoadIdentity();
  
  /* translation and rotation by mouse */
  glTranslatef(shiftX, shiftY, 0);
  float s = (float)sqrt(qx*qx + qy*qy + qz*qz);
  if(s != 0)
    glRotatef(2.0f*(float)acos(qw)*180/3.1412f, qx/s, qy/s,qz/s);
  
  /* render the mesh */
//  flatShading
	flatShading();
  if(edge)
    drawMesh();
  
  glFlush(); 
  glutSwapBuffers(); 
} 

void myReshape(int w, int h){
  width = w;
  height = h;
  
  glViewport(0, 0, w, h); 
  glMatrixMode(GL_PROJECTION); 
  glLoadIdentity(); 
  if (w <= h) 
    glOrtho(-2.0/zoom, 2.0/zoom,
            -2.0 * (GLfloat) h / (GLfloat) w /zoom, 
            2.0 * (GLfloat) h / (GLfloat) w /zoom,
            -10.0, 10.0); 
  else 
    glOrtho(-2.0 * (GLfloat) w / (GLfloat) h / zoom, 
            2.0 * (GLfloat) w / (GLfloat) h /zoom,
            -2.0/zoom, 2.0/zoom, -10.0, 10.0); 
  glMatrixMode(GL_MODELVIEW); 
} 

void myinit(){
  GLfloat mat_specular[]={0.5, 0.5, 0.5, 1.0};
  GLfloat mat_diffuse[]={0.5, 0.5, 1.0, 1.0};
  GLfloat mat_ambient[]={0.5, 0.5, 0.5, 1.0}; 
  GLfloat mat_shininess={100.0};
  GLfloat light_ambient[]={0.1, 0.1, 0.1, 1.0};
  GLfloat light_diffuse[]={1.0, 1.0, 1.0, 1.0};
  GLfloat light_specular[]={1.0, 1.0, 1.0, 1.0};
  
  /* set up ambient, diffuse, and specular components for light 0 */
  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
  
  /* define material proerties for front face of all polygons */
  glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
  glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
  glMaterialf(GL_FRONT, GL_SHININESS, mat_shininess);
  
  glShadeModel(GL_SMOOTH);   /* smooth shading */
  glEnable(GL_LIGHTING); /* enable lighting */
  glEnable(GL_LIGHT0);   /* enable light 0 */
  glEnable(GL_DEPTH_TEST); /* enable z buffer */
  glDepthFunc(GL_LEQUAL);
  
  glClearColor (1.0, 1.0, 1.0, 1.0);
  
  glColor3f (0.0, 0.0, 0.0);
  glLineWidth(10.0f);
}

void mouse(int btn, int state, int x, int y){ 
  /* mouse callback, selects an axis about which to rotate */
  if(state == GLUT_DOWN){
    mouseX = x;
    mouseY = y;
  }
  else if(btn==GLUT_LEFT_BUTTON){
    float mx = 0.0025f*(x - mouseX)/zoom;
    float my = 0.0025f*(y - mouseY)/zoom;
    
    //rotation
    float c = (float)cos(my);
    float s = (float)sin(my);
    
    float rw = c*qw - s*qx;
    float rx = c*qx + s*qw;
    float ry = c*qy - s*qz;
    float rz = c*qz + s*qy;
    
    c = (float)cos(mx);
    s = (float)sin(mx);
    
    qw = c*rw - s*ry;
    qx = c*rx + s*rz;
    qy = c*ry + s*rw;
    qz = c*rz - s*rx;
    
    float n = (float)sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
    if(n != 0){
      qw /= n;
      qx /= n;
      qy /= n;
      qz /= n;
    }
    else{
      qw = 1.0f;
      qx = qy = qz = 0.0f;
    }
    display();
  }
  else if(btn==GLUT_RIGHT_BUTTON){
    zoom -= 0.0025f*(y - mouseY);
    if(zoom > 20.0f) zoom = 20.0f;
    else if(zoom < 0.05f) zoom = 0.05f;
    myReshape(width, height);
    display();
  }
  else if(btn==GLUT_MIDDLE_BUTTON){
    shiftX += 0.01f*(x - mouseX)/zoom;
    shiftY -= 0.01f*(y - mouseY)/zoom;
    display();
  }
} 

void allocateMem(){

  vertices0 =  new float[numVertices*20][3];
  triangles0 = new int[numTriangles*10][3];

  vertices =  new float[numVertices][3];
  triangles = new int[numTriangles][3];
  vertexNormals = new float[numVertices][3];
  faceNormals = new float[numTriangles*10][3];
}

void deleteMesh(){

  delete[] vertices0;
  delete[] triangles0;

  delete[] vertices;
  delete[] triangles;
  delete[] faceNormals;
  delete[] vertexNormals;
}

void readMesh(char* file_name){
  /* Read the mesh file */
  FILE* in = fopen(file_name, "r");
  
  fscanf(in, "%d", &numVertices);
  fscanf(in, "%d", &numTriangles);
  
  allocateMem();
  
  for(int i=0; i<numVertices; i++){
    float* v = vertices[i];
    fscanf(in, "%f %f %f", &v[0], &v[1], &v[2]);
  }
  
  for( i=0; i<numTriangles; i++){
    int* t = triangles[i];
    fscanf(in, "%d %d %d", &t[0], &t[1], &t[2]);
  }
  
  fclose(in);
}
  
void normalizeMeshSize(float radius){
  // Compute the bounding box
  float max[3], min[3];
  float size[3], mid[3];
  for(int i=0; i<3; i++){
    max[i] = vertices[0][i];
    min[i] = vertices[0][i];
    for(int j=1; j<numVertices; j++){
      float v = vertices[j][i];
      if(v > max[i])
        max[i] = v;
      else if(v < min[i])
        min[i] = v;
    }
    size[i] = max[i] - min[i];
    mid[i] = 0.5f*(max[i] + min[i]);
  }
  
  float diagonal = (float)sqrt(size[0]*size[0] + size[1]*size[1] + size[2]*size[2]);
  float scale = radius/(2.0f*diagonal);
  
  for(i=0; i<3; i++)
    for(int j=0; j<numVertices; j++)
      vertices[j][i] = scale*(vertices[j][i] - mid[i]);
}

void keyboard(unsigned char key, int x, int y){
  switch (key) {
  case 'e':
    edge = !edge;

    display();
    break;
  }
}

int main(int argc, char** argv){
  /* setup view data */
  qw = 1;
  qx = qy =  qz = 0;
  zoom = 1;
  shiftX = shiftY = 0;
  edge = false;
  
  /* read mesh data */
  readMesh("Armadillo.mesh");
  
  /* rescale mesh for fitting the view */
  normalizeMeshSize(10.0f);
  
  computeVertexNormals();
  usePN();
  computeFaceNormals();
  
  glutInit(&argc, argv); 
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH); 
  glutInitWindowSize(500, 500); 
  glutCreateWindow("smooth");
  myinit();
  glutReshapeFunc(myReshape); 
  glutDisplayFunc(display); 
  glutMouseFunc(mouse); 
  glutKeyboardFunc(keyboard);
  
  glutMainLoop(); 
  
  deleteMesh();
  
  return 0;
}

