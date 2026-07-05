#include <tinyxml2.h>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <vector>
#include <boost/algorithm/string.hpp>
#include <sstream>

// tutorial demo program
//#include <stdafx.h>

using namespace tinyxml2;

// ----------------------------------------------------------------------
// STDOUT dump and indenting utility functions
// ----------------------------------------------------------------------
const unsigned int NUM_INDENTS_PER_SPACE=2;
std::ofstream ofs("test.txt");

// simple quaternion to replace tf::Quaternion (keeps this tool ROS-free)
struct SimpleQuaternion
{
  double x_, y_, z_, w_;
  void setRPY(double roll, double pitch, double yaw)
  {
    double halfYaw = yaw * 0.5;
    double halfPitch = pitch * 0.5;
    double halfRoll = roll * 0.5;
    double cosYaw = std::cos(halfYaw);
    double sinYaw = std::sin(halfYaw);
    double cosPitch = std::cos(halfPitch);
    double sinPitch = std::sin(halfPitch);
    double cosRoll = std::cos(halfRoll);
    double sinRoll = std::sin(halfRoll);
    x_ = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    y_ = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    z_ = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
    w_ = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
  }
  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double w() const { return w_; }
};

const char * getIndent( unsigned int numIndents )
{
  static const char * pINDENT="                                      + ";
  static const unsigned int LENGTH=strlen( pINDENT );
  unsigned int n=numIndents*NUM_INDENTS_PER_SPACE;
  if ( n > LENGTH ) n = LENGTH;

  return &pINDENT[ LENGTH-n ];
}

// same as getIndent but no "+" at the end
const char * getIndentAlt( unsigned int numIndents )
{
  static const char * pINDENT="                                        ";
  static const unsigned int LENGTH=strlen( pINDENT );
  unsigned int n=numIndents*NUM_INDENTS_PER_SPACE;
  if ( n > LENGTH ) n = LENGTH;

  return &pINDENT[ LENGTH-n ];
}

int dump_attribs_to_stdout(XMLElement* pElement, unsigned int indent)
{
  if ( !pElement ) return 0;

  const XMLAttribute* pAttrib=pElement->FirstAttribute();
  int i=0;
  int ival;
  double dval;
  const char* pIndent=getIndent(indent);
  printf("\n");
  while (pAttrib)
    {
      printf( "%s%s: value=[%s]", pIndent, pAttrib->Name(), pAttrib->Value());

      if (pAttrib->QueryIntValue(&ival)==XML_SUCCESS)    printf( " int=%d", ival);
      if (pAttrib->QueryDoubleValue(&dval)==XML_SUCCESS) printf( " d=%1.1f", dval);
      printf( "\n" );
      i++;
      pAttrib=pAttrib->Next();
    }
  return i;
}

void dump_include_model( XMLNode* pParent, unsigned int indent = 0 ){
  XMLNode* pChild;
  std::cout << "-" << std::endl;
  for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
    {
      std::string type = pChild->Value();
      //std::cout << pChild->Value() << std::endl;

      if(type == "uri"){

	//std::cout << "uuuuuuuuuuurrrrrrrrrrrrrrrriiiiiiiiiiiiI" << std::endl;
	std::cout << getIndentAlt(indent) << "model: \"" << pChild->FirstChild()->Value() << "\"" << std::endl;
      }else if(type == "name"){
	//std::cout << "namerrrrrrrrrrrriiiiiiiiiiiiI" << std::endl;
	std::cout << getIndentAlt(indent) << "name: \"" << pChild->FirstChild()->Value() << "\"" << std::endl;
      }else if(type == "pose"){
	std::string child_value = pChild->FirstChild()->Value();
	std::vector<std::string> v;
	boost::algorithm::split( v, child_value, boost::algorithm::is_space() );
	//std::cout << getIndentAlt(indent) << "pose: " << pChild->FirstChild()->Value() << std::endl;
	std::cout << getIndentAlt(indent) << "pose:" << std::endl;
	std::cout << getIndentAlt(indent + 1) << "position:" << std::endl;
	std::cout << getIndentAlt(indent + 2) << "x: " << v[0] << std::endl;
	std::cout << getIndentAlt(indent + 2) << "y: " << v[1] << std::endl;
	std::cout << getIndentAlt(indent + 2) << "z: " << v[2] << std::endl;

	std::istringstream r_str(v[3]);
	int r,p,y;
	r_str >> r;
	std::istringstream p_str(v[4]);
	p_str >> p;
	std::istringstream y_str(v[5]);
	y_str >> y;

	SimpleQuaternion q;
	q.setRPY(r,p,y);
	std::cout << getIndentAlt(indent + 1) << "orientation:" << std::endl;
	std::cout << getIndentAlt(indent + 2) << "x: " << q.x() << std::endl;
	std::cout << getIndentAlt(indent + 2) << "y: " << q.y() << std::endl;
	std::cout << getIndentAlt(indent + 2) << "z: " << q.z() << std::endl;
	std::cout << getIndentAlt(indent + 2) << "w: " << q.w() << std::endl;

      }
    }
  std::cout << getIndentAlt(indent) << "frmae-id: \"map\"" << std::endl;
  std::cout << getIndentAlt(indent) << "robot: false" << std::endl;
}


void dump_to_stdout( XMLNode* pParent, unsigned int indent = 0 )
{
  if ( !pParent ) return;

  XMLNode* pChild;
  std::string value = pParent->Value() ? pParent->Value() : "";
  if ( value == "include"){
    dump_include_model(pParent, 1);
  }

  for ( pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
    {
      dump_to_stdout( pChild, indent+1 );
    }
}

void dump_to_stdout(const char* pFilename)
{
  XMLDocument doc;
  bool loadOkay = (doc.LoadFile(pFilename) == XML_SUCCESS);
  if (loadOkay)
    {
      printf("\n%s:\n", pFilename);
      dump_to_stdout( &doc ); // defined later in the tutorial
    }
  else
    {
      printf("Failed to load file \"%s\"\n", pFilename);
    }
}



int main(int argc, char** argv){
  (void)argc;
  (void)argv;

  //ofs<<"aa"<<std::endl;
  dump_to_stdout("vrc_final_task1.world");

  return 0;


}
