/*
* MADE BY SREENIVAS git new
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/*
 * Base code for CS 251 Software Systems Lab
 * Department of Computer Science and Engineering, IIT Bombay
 *
 */


#include "cs251_base.hpp"
#include "render.hpp"
#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs251
{
  /**  The is the constructor
   * This is the documentation block for the constructor.
   */

  dominos_t::dominos_t()
 {
    //Ground
    /*! \var b1
     * \brief pointer to the body ground
     */
    b2Body* b1;
   {

      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
//==========================================================================------------------------------------sreenivas
//==========================================================================-------------------------Left part
  int x_sreenivas,y_sreenivas;
      x_sreenivas = 0; y_sreenivas = 3;  
      //overall top for colliding

    {
      b2PolygonShape shape;
      shape.SetAsBox(16.0f, 0.25f);

      b2BodyDef bd;
      bd.position.Set(0.0+x_sreenivas, 48.0f+y_sreenivas);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
       
       // falling sphere left

    {
       b2Body* spherebody;

       b2CircleShape circle;
       circle.m_radius = 1;

       b2FixtureDef ballfd;
       ballfd.shape = &circle;
       ballfd.density = 35.0f;
       ballfd.friction = 0.0f;
       ballfd.restitution = 0.0f;
       b2BodyDef ballbd;
       ballbd.type = b2_dynamicBody;
       ballbd.position.Set(-38.0f+x_sreenivas, 36.0f+y_sreenivas);
       spherebody = m_world->CreateBody(&ballbd);
       spherebody->CreateFixture(&ballfd);

    }

       //Upper horizontal shelf left

    {
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), -0.2f);

      b2BodyDef bd;
      bd.position.Set(-8.0f+x_sreenivas, 10.0f+y_sreenivas);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

      //the revolving platform left

    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-17+x_sreenivas,15+y_sreenivas);
      bd->fixedRotation =false;//if set false box revolves about c.m when a ball falls into it

      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 1.0;
      fd2->restitution = 1.0f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(6,0.3, b2Vec2(2.0f,0.f), 0.0f);
      fd2->shape = &bs2;

      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd2);

      /// The hook - the platform should revolve around this object left

      b2BodyDef bd0;
      bd0.position.Set(-15+x_sreenivas, 15+y_sreenivas);
      b2Body* hook = m_world->CreateBody(&bd0);

      b2PolygonShape shape;
      shape.SetAsBox(0.3, 0.3);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 0.1f;
      hook->CreateFixture(&fd);

      ///very light ball left on the platform

      b2CircleShape circle;
      circle.m_radius = 0.8;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 0.05f;
      ballfd.friction = 10.0f;
      ballfd.restitution = 0.0f;

	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(-19+x_sreenivas, 16+y_sreenivas);
	  b2Body* spherebody;
	  spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);

      /*! Revolute joint */ //left

      b2RevoluteJointDef jd;
      jd.Initialize(hook, box1, b2Vec2(-15+x_sreenivas,15+y_sreenivas)); // The last parameter is the center of rotation
      jd.collideConnected = false;
      jd.enableLimit = false;  // No need for limits
      m_world->CreateJoint(&jd);
    }

      //the base on which dominos rest left

    {
        b2PolygonShape shape,shape1;
        shape.SetAsBox(5.00f, 0.25f);
        shape1.SetAsBox(1.00,0.25,b2Vec2(0,0),-0.6f);
        b2BodyDef bd,bd1;
        bd.position.Set(27.5f+x_sreenivas, 20.0f+y_sreenivas);
        bd1.position.Set(22.0f+x_sreenivas, 20.5f+y_sreenivas);
        b2Body* ground = m_world->CreateBody(&bd);
        b2Body* ground1 = m_world->CreateBody(&bd1);
        ground->CreateFixture(&shape, 0.0f);
        ground1->CreateFixture(&shape1, 0.0f);
    }

      //dominos  left

    {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.0f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 2.0f;
        fd.friction = 1.0f;
        fd.restitution = 0.0f;

        for (int i = 0; i < 10; ++i)
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(x_sreenivas+32.0f - 1.0f * i, 21.25f+y_sreenivas);
            b2Body* body = m_world->CreateBody(&bd);
            body->CreateFixture(&fd);
        }
    }

       // tray of spheres platform left

    {
        b2PolygonShape shape,shape1;
        shape.SetAsBox(4.00f, 0.25f);
        shape1.SetAsBox(1.00,0.25,b2Vec2(0,0),-0.6f);
        b2BodyDef bd,bd1;
        bd.position.Set(-31.5f+x_sreenivas, 14.25f+y_sreenivas);
        bd1.position.Set(-36.0f+x_sreenivas, 14.75f+y_sreenivas);
        b2Body* ground = m_world->CreateBody(&bd);
        b2Body* ground1 = m_world->CreateBody(&bd1);
        ground->CreateFixture(&shape, 0.0f);
        ground1->CreateFixture(&shape1, 0.0f);
    }

       //  left tray of spheres

   {
        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 0.75;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = 2.0f;
        ballfd.friction = 0.0f;
        ballfd.restitution = 0.0f;

        for (int i = 0; i < 3; ++i)
        {
            b2BodyDef ballbd;
            ballbd.type = b2_dynamicBody;
            ballbd.position.Set(x_sreenivas+-32.2f + i*2.0, 15.25f+y_sreenivas);
            spherebody = m_world->CreateBody(&ballbd);
            spherebody->CreateFixture(&ballfd);
        }
    }


//===========================================================----------------------------------------     right part sreenivas

      //falling sphere right

    {
       b2Body* spherebody;

       b2CircleShape circle;
       circle.m_radius = 1;

       b2FixtureDef ballfd;
       ballfd.shape = &circle;
       ballfd.density = 35.0f;
       ballfd.friction = 0.0f;
       ballfd.restitution = 0.0f;

       b2BodyDef ballbd;
       ballbd.type = b2_dynamicBody;
       ballbd.position.Set(38.0f+x_sreenivas, 36.6f+y_sreenivas);
       spherebody = m_world->CreateBody(&ballbd);
       spherebody->CreateFixture(&ballfd);

    }

    //Upper horizontal shelf  right

    {
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(20.f,20.f), 0.2f);

      b2BodyDef bd;
      bd.position.Set(8.0f+x_sreenivas, 10.0f+y_sreenivas);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
   
   // revolving platform right

    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(17+x_sreenivas,15+y_sreenivas);
      bd->fixedRotation =false;//if set false box revolves about c.m when a ball falls into it

      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 10.0;
      fd2->restitution = 1.0f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(6,0.3, b2Vec2(-2.0f,0.f), 0.0f);
      fd2->shape = &bs2;

      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd2);

      // The hook - the platform should revolve around this object

      b2BodyDef bd0;
      bd0.position.Set(15+x_sreenivas, 15+y_sreenivas);
      b2Body* hook = m_world->CreateBody(&bd0);

      b2PolygonShape shape;
      shape.SetAsBox(0.3, 0.3);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 0.1f;
      hook->CreateFixture(&fd);

      //very light ball  right on the platform

      b2CircleShape circle;
      circle.m_radius = 0.8;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 0.05f;
      ballfd.friction = 10.0f;
      ballfd.restitution = 0.0f;

	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(19+x_sreenivas, 16+y_sreenivas);
	  b2Body* spherebody;
	  spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);

      /*! Revolute joint */

      b2RevoluteJointDef jd;
      jd.Initialize(hook, box1, b2Vec2(15+x_sreenivas,15+y_sreenivas)); // The last parameter is the center of rotation
      jd.collideConnected = false;
      jd.enableLimit = false;  // No need for limits
      m_world->CreateJoint(&jd);

    }

    //dominos resting platform right

    {
        b2PolygonShape shape,shape1;
        shape.SetAsBox(5.00f, 0.25f);
        shape1.SetAsBox(1.00,0.25,b2Vec2(0,0),0.6f);
        b2BodyDef bd,bd1;
        bd.position.Set(-27.5f+x_sreenivas, 20.0f+y_sreenivas);     
        bd1.position.Set(-22.0f+x_sreenivas, 20.5f+y_sreenivas);
        b2Body* ground = m_world->CreateBody(&bd);
        b2Body* ground1 = m_world->CreateBody(&bd1);
        ground->CreateFixture(&shape, 0.0f);
        ground1->CreateFixture(&shape1, 0.0f);
    }

      //dominos right

    {
        b2PolygonShape shape;
        shape.SetAsBox(0.1f, 1.0f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 2.0f;
        fd.friction = 1.0f;
        fd.restitution = 0.0f;

        for (int i = 0; i < 10; ++i)
        {
            b2BodyDef bd;
            bd.type = b2_dynamicBody;
            bd.position.Set(x_sreenivas+-32.0f + 1.0f * i, 21.25f+y_sreenivas);
            b2Body* body = m_world->CreateBody(&bd);
            body->CreateFixture(&fd);
        }
    }

     // spheres resting platform right

   {
        b2PolygonShape shape,shape1;
        shape.SetAsBox(4.00f, 0.25f);
        shape1.SetAsBox(1.00,0.25,b2Vec2(0,0),0.6f);
        b2BodyDef bd,bd1;
        bd.position.Set(31.5f+x_sreenivas, 14.25f+y_sreenivas);
        bd1.position.Set(36.0f+x_sreenivas, 14.75f+y_sreenivas);
        b2Body* ground = m_world->CreateBody(&bd);
        b2Body* ground1 = m_world->CreateBody(&bd1);
        ground->CreateFixture(&shape, 0.0f);
        ground1->CreateFixture(&shape1, 0.0f);
    }

      // tray of spheres right

   {
        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 0.75;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = 2.0f;
        ballfd.friction = 0.0f;
        ballfd.restitution = 0.0f;

        for (int i = 0; i < 3; ++i)
        {
            b2BodyDef ballbd;
            ballbd.type = b2_dynamicBody;
            ballbd.position.Set(x_sreenivas+32.2f - i*2.0, 15.25f+y_sreenivas);
            spherebody = m_world->CreateBody(&ballbd);
            spherebody->CreateFixture(&ballfd);
        }
    }

    ///CODE FOR CONVEYOR BELTS ******************************************

    {
        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius =0.75f;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = 1.0f;
        ballfd.friction = 10.0f;
        ballfd.restitution = 0.0f;

        b2BodyDef ballbd;
        ballbd.type = b2_dynamicBody;
        ballbd.position.Set(8.0f+x_sreenivas,26.0f+y_sreenivas);
        spherebody = m_world->CreateBody(&ballbd);
	    spherebody->CreateFixture(&ballfd);

        b2PolygonShape shape2;
        shape2.SetAsBox(0.2f, 2.0f);
        b2BodyDef bd2;
        bd2.position.Set(8.0f+x_sreenivas, 26.0f+y_sreenivas);
        b2Body* body2 = m_world->CreateBody(&bd2);

        b2RevoluteJointDef jointDef;
        jointDef.maxMotorTorque = 10.0f;
        jointDef.motorSpeed = 15.11f;           //for right one
        jointDef.enableMotor = true;
        jointDef.bodyA = spherebody;
        jointDef.bodyB = body2;
        jointDef.localAnchorA.Set(0,0);
        jointDef.localAnchorB.Set(0,0);
        m_world->CreateJoint(&jointDef);

        ballbd.position.Set(-8.0f+x_sreenivas,26.0f+y_sreenivas);
        spherebody = m_world->CreateBody(&ballbd);
	    spherebody->CreateFixture(&ballfd);
	    bd2.position.Set(-8.0f+x_sreenivas, 26.0f+y_sreenivas);
        body2 = m_world->CreateBody(&bd2);
        jointDef.maxMotorTorque = 10.0f;
        jointDef.motorSpeed = -15.0f;              //for left one
        jointDef.enableMotor = true;
        jointDef.bodyA = spherebody;
        jointDef.bodyB = body2;
        jointDef.localAnchorA.Set(0,0);
        jointDef.localAnchorB.Set(0,0);
        m_world->CreateJoint(&jointDef);

    }
    
     /// RIGHT WHEEL OF CONVEYER BELT
  
    {
        b2Body* spherebody;
        b2CircleShape circle;
        circle.m_radius =0.75;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = 1.0f;
        ballfd.friction = 10.0f;
        ballfd.restitution = 0.0f;

        b2BodyDef ballbd;
        ballbd.type = b2_dynamicBody;
        ballbd.position.Set(11.9f+x_sreenivas,26.0f+y_sreenivas);
        spherebody = m_world->CreateBody(&ballbd);
	    spherebody->CreateFixture(&ballfd);

        b2PolygonShape shape2;
        shape2.SetAsBox(0.2f, 2.0f);
        b2BodyDef bd2;
        bd2.position.Set(11.9f+x_sreenivas, 26.0f+y_sreenivas);
        b2Body* body2 = m_world->CreateBody(&bd2);
        b2RevoluteJointDef jointDef;
        jointDef.maxMotorTorque = 10.0f;
        jointDef.motorSpeed = 15.11f;       //for right one
        jointDef.enableMotor = true ; 
        jointDef.bodyA = spherebody;
        jointDef.bodyB = body2;
        jointDef.localAnchorA.Set(0,0);
        jointDef.localAnchorB.Set(0,0);
        m_world->CreateJoint(&jointDef);

        ///Right wheel of left conveyer belt

        ballbd.position.Set(-11.9f+x_sreenivas,26.0f+y_sreenivas);
	    spherebody = m_world->CreateBody(&ballbd);
	    spherebody->CreateFixture(&ballfd);
        bd2.position.Set(-11.9f+x_sreenivas, 26.0f+y_sreenivas);
        body2 = m_world->CreateBody(&bd2);
        jointDef.maxMotorTorque = 10.0f;
        jointDef.motorSpeed = -15.0f;         //for left one
        jointDef.enableMotor = true ;
        jointDef.bodyA = spherebody;
        jointDef.bodyB = body2;
        jointDef.localAnchorA.Set(0,0);
        jointDef.localAnchorB.Set(0,0);
        m_world->CreateJoint(&jointDef);
    }
    
     /// BUILDING BLOCKS OF RIGHT CONVEYER BELT
   
    {
      b2Body* box1,*box1x;
      b2Body* hook;
      for(int i=0;i<10;i++){
      if(i==0){
      b2BodyDef *bd = new b2BodyDef;b2BodyDef *bdx = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(x_sreenivas+7.0+1.3*(i+1),27.0+y_sreenivas);
      bd->fixedRotation = false;//if set false box revolves about c.m when a ball falls into it
      bdx = bd;
      b2FixtureDef *fd2 = new b2FixtureDef;b2FixtureDef *fd2x = new b2FixtureDef;
      fd2->density = 0.01;
      fd2->friction = 10.0;
      fd2->restitution = 0.0f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.75,0.2);
      fd2->shape = &bs2;
      fd2x = fd2;
      box1 = m_world->CreateBody(bd);box1x = m_world->CreateBody(bdx);
      box1->CreateFixture(fd2);box1x->CreateFixture(fd2x);
      b2RevoluteJointDef jdx;
      jdx.collideConnected = true;
      jdx.Initialize(box1,box1x, b2Vec2(7.0+1.3*(i+1),27.0+y_sreenivas));

      m_world->CreateJoint(&jdx);
      continue;
    }

      /// The hook - the continuation of conveyer

      if(i>0){
      b2BodyDef bd0;
      bd0.type = b2_dynamicBody;
      if(i<4){
      bd0.position.Set(x_sreenivas+7.0+1.3*(i+1),27.0+y_sreenivas);
             }
      if(i==4){bd0.position.Set(x_sreenivas+7.0+1.3*(4)+0.8,26.25+y_sreenivas);}
      if(i>4&&i<9){bd0.position.Set(x_sreenivas+7.0+1.3*(9-i),25.5f+y_sreenivas);}
      if(i==9){bd0.position.Set(x_sreenivas+8.3-0.75,26.25+y_sreenivas);}
      hook = m_world->CreateBody(&bd0);
      b2PolygonShape shape;
      if(i!=4 && i!=9){
      shape.SetAsBox(0.75,0.2);
             }
      else{shape.SetAsBox(0.2,0.75);}

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 0.01f;
      fd.friction = 10.0;
      fd.restitution=0.0f;
      hook->CreateFixture(&fd);

      b2RevoluteJointDef jd;
      if(i<=4){

      jd.Initialize(hook,box1, b2Vec2(7.0+1.3*(i+0.5),27.0f+y_sreenivas)); /// The last parameter is the center of rotation
      }
      if(i>4&&i<=9){

                  jd.Initialize(hook,box1, b2Vec2(7.0+1.3*(9.5-i),25.5f+y_sreenivas));jd.collideConnected = true;

                    }
      if(i==9){

            b2RevoluteJointDef jdloop;
            jdloop.Initialize(hook,box1x, b2Vec2(8.3-0.75,27.0f+y_sreenivas));
            jdloop.collideConnected = true;
            m_world->CreateJoint(&jdloop);
      }
      m_world->CreateJoint(&jd);
      }
      box1=hook;
     }
    }
 
     /// BUILDING BLOCKS OF LEFT CONVEYER BELT
  
    {
      b2Body* box1,*box1x;
      b2Body* hook;
      for(int i=0;i<10;i++){
      if(i==0){
      b2BodyDef *bd = new b2BodyDef;b2BodyDef *bdx = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(x_sreenivas-7.0-1.3*(i+1),27.0+y_sreenivas);
      bd->fixedRotation = false;//if set false box revolves about c.m when a ball falls into it
      bdx = bd;
      b2FixtureDef *fd2 = new b2FixtureDef;b2FixtureDef *fd2x = new b2FixtureDef;
      fd2->density = 0.01;
      fd2->friction = 10.0;
      fd2->restitution = 0.0f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.75,0.2);
      fd2->shape = &bs2;
      fd2x = fd2;
      box1 = m_world->CreateBody(bd);box1x = m_world->CreateBody(bdx);
      box1->CreateFixture(fd2);box1x->CreateFixture(fd2x);
      b2RevoluteJointDef jdx;
      jdx.collideConnected = true;
      jdx.Initialize(box1,box1x, b2Vec2(-7.0-1.3*(i+1),27.0+y_sreenivas));

      m_world->CreateJoint(&jdx);
      continue;
    }

      /// The hook - the continuation of conveyer

      if(i>0){
      b2BodyDef bd0;
      bd0.type = b2_dynamicBody;
      if(i<4){
      bd0.position.Set(x_sreenivas-7.0-1.3*(i+1),27.0+y_sreenivas);
      }
      if(i==4){bd0.position.Set(x_sreenivas-7.0-1.3*(4)-0.8,26.25+y_sreenivas);}
      if(i>4&&i<9){bd0.position.Set(x_sreenivas-7.0-1.3*(9-i),25.5f+y_sreenivas);}
      if(i==9){bd0.position.Set(x_sreenivas-8.3+0.75,26.25+y_sreenivas);}
      hook = m_world->CreateBody(&bd0);
      b2PolygonShape shape;
      if(i!=4 && i!=9){
      shape.SetAsBox(0.75,0.2);
             }
      else{shape.SetAsBox(0.2,0.75);}
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 0.01f;
      fd.friction = 10.0;
      fd.restitution=0.0f;
      hook->CreateFixture(&fd);

      b2RevoluteJointDef jd;
      if(i<=4){

      jd.Initialize(hook,box1, b2Vec2(-7.0-1.3*(i+0.5),27.0f+y_sreenivas)); /// The last parameter is the center of rotation
      }
      if(i>4&&i<=9){

               jd.Initialize(hook,box1, b2Vec2(-7.0-1.3*(9.5-i),25.5f+y_sreenivas));jd.collideConnected = true;
                    }
      if(i==9){

            b2RevoluteJointDef jdloop;
            jdloop.Initialize(hook,box1x, b2Vec2(-8.3+0.75,27.0f+y_sreenivas));
            jdloop.collideConnected = true;
            m_world->CreateJoint(&jdloop);
      }
      m_world->CreateJoint(&jd);
      }
      box1=hook;
     }
    }

        //Bowling pin at bottom

     {
        float a,b,x0;
        a=2.0;b=2.0;x0=-1;
        b2FixtureDef *fd1 = new b2FixtureDef;
        fd1->friction = 0.8;
        fd1->restitution = 0.1f;
        fd1->shape = new b2PolygonShape;
        b2PolygonShape bs1;
        b2BodyDef *bd = new b2BodyDef;
        bd->type = b2_dynamicBody;
        bd->position.Set(30,1.0);
        bd->fixedRotation = false;
        b2Body* box1 = m_world->CreateBody(bd);
        for (float theta = -0.2; theta < 0.7; theta = theta + 0.02)
        {
            float x = ((a+b*tan(theta))+sqrt(pow(a+b*tan(theta) , 2.0) + pow(x0,2.0) - 2*a*x0))*(pow(cos(theta) , 2.0));
            float y = x*tan(theta);
            bs1.SetAsBox(0.1,0.1,b2Vec2(x,y),0.0);
            fd1->shape = &bs1;
            box1->CreateFixture(fd1);
        }

        a=0.0;b=1.0;x0=1.0;
        for (float theta = 0; theta < 1.0; theta = theta + 0.02)
        {
            float x = ((a+b*tan(theta))+sqrt(pow(a+b*tan(theta) , 2.0) + pow(x0,2.0) - 2*a*x0))*(pow(cos(theta) , 2.0));
            float y = x*tan(theta);
            bs1.SetAsBox(0.1,0.1,b2Vec2(-1.0*x+5.4,y+4.1),0.0);
            fd1->shape = &bs1;
            box1->CreateFixture(fd1);
        }

        a=2.0;b=2.0;x0=-1;
        for (float theta = -0.2; theta < 0.7; theta = theta + 0.02)
        {
            float x = ((a+b*tan(theta))+sqrt(pow(a+b*tan(theta) , 2.0) + pow(x0,2.0) - 2*a*x0))*(pow(cos(theta) , 2.0));
            float y = x*tan(theta);
            bs1.SetAsBox(0.1,0.1,b2Vec2(-x+7.0,y),0.0);
            fd1->shape = &bs1;
            box1->CreateFixture(fd1);
        }

        a=0.0;b=1.0;x0=1.0;
        for (float theta = 0; theta < 1.0; theta = theta + 0.02)
        {
            float x = -1*((a+b*tan(theta))+sqrt(pow(a+b*tan(theta) , 2.0) + pow(x0,2.0) - 2*a*x0))*(pow(cos(theta) , 2.0));
            float y = -1*x*tan(theta);
            bs1.SetAsBox(0.1,0.1,b2Vec2(-1.0*x+1.4,y+4.1),0.0);
            fd1->shape = &bs1;
            box1->CreateFixture(fd1);
        }

        a=0.0;b=-1.0;x0=1.0;
        for (float theta = -0.2; theta < 3.15; theta = theta + 0.02)
        {
            float x = cos(theta);
            float y = sin(theta);
            bs1.SetAsBox(0.1,0.1,b2Vec2(x+3.4,y+6.0),0.0);
            fd1->shape = &bs1;
            box1->CreateFixture(fd1);
        }

        for (float theta = 0.1; theta < 1.9; theta = theta + 0.02)
        {
            float x = theta;
            float y = 0;
            bs1.SetAsBox(0.1,0.1,b2Vec2(x+2.5,y-0.8),0.0);
            fd1->shape = &bs1;
            box1->CreateFixture(fd1);
        }
      }

//========================================================-------------------------------------------sreenivas code ends
//========================================================-------------------------------------------rana code

     int y_rana ;
     y_rana   = 19;
     int x_rana ;
     x_rana = 10;

    //The middle pulley system

    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-10+x_rana,13+y_rana);
      bd->fixedRotation = true;

      //The open box

      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 20.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 20.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 20.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;

      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      bs1.SetAsBox(0.632,0.632, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;

      //The bar1

      bd->position.Set(-14.5+x_rana,12+y_rana);
      fd1->friction = 10.0f;
      fd1->density = 30.0;
      b2Body* box3 = m_world->CreateBody(bd);
      box3->CreateFixture(fd1);

      //The bar2

      bd->position.Set(-5.5+x_rana,12+y_rana);
      fd1->friction = 10.0f;
      fd1->density = 30.0;
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

     // The pulley joint1

      b2PulleyJointDef* joint = new b2PulleyJointDef();
      b2Vec2 worldAnchorGround3(-10+x_rana, 20+y_rana); // Anchor point for ground 2 in world axis
      b2Vec2 worldAnchorGround11(-14.5+x_rana, 20+y_rana); // Anchor point for ground 1 in world axis
      float32 ratio = 1.0f; // Define ratio
      joint->Initialize(box1, box3, worldAnchorGround3, worldAnchorGround11, box1->GetWorldCenter(), box3->GetWorldCenter(), ratio);
      m_world->CreateJoint(joint);

      // The pulley joint2

      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorGround1(-10+x_rana, 20+y_rana); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(-5.5+x_rana, 20+y_rana); // Anchor point for ground 2 in world axis
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);

    }

    //The small revolving horizontal platform  in middle

    {
      b2PolygonShape shape;
      shape.SetAsBox(1.8f, 0.2f);

      b2BodyDef bd;
      bd.position.Set(-16.6f+x_rana, 12.0f+y_rana);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2BodyDef bd2;
      bd2.position.Set(-16.6f+x_rana, 14.0f+y_rana);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);

/*-------------------------------------------------------*/

      bd.position.Set(-3.4f+x_rana, 12.0f+y_rana);
      body = m_world->CreateBody(&bd);
      body->CreateFixture(fd);

      bd2.position.Set(-3.4f+x_rana, 14.0f+y_rana);
      body2 = m_world->CreateBody(&bd2);

      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);

    }

    //The heavy sphere on the small revolving platform in middle

    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 0.5;
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 0.1f;
      ballfd.friction = 10.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;

      ballbd.position.Set(-3.4f+x_rana, 18.0f+y_rana);
      ballbd.fixedRotation =false;
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);

      ballbd.position.Set(-16.6f+x_rana, 18.0f+y_rana);
      ballbd.fixedRotation =false;
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);

    }


//===========================================================---------------------------------------------------rana code ends
//===========================================================---------------------------------------------------srinath code
//===========================================================------------------------Left part
    int x_srinath,y_srinath;
    x_srinath = 0; y_srinath = -5; 

 //Top pendulum horizontal shelf left 

   {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(-38.0f+x_sreenivas, 35.0f+y_sreenivas);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

 //The pendulum that knocks the ball left

    {
      b2Body* b2;
      {
	   b2PolygonShape shape;
	   shape.SetAsBox(1.5f, 0.25f);
	  
	   b2BodyDef bd;
	   bd.position.Set(-38.0f+x_sreenivas, 43.0f+y_sreenivas);
	   b2 = m_world->CreateBody(&bd);
	   b2->CreateFixture(&shape, 10.0f);
      }
	
      b2Body* b4;
      {
	    b2PolygonShape shape;
	    shape.SetAsBox(1.0f, 1.0f);
	  
	    b2BodyDef bd;
	    bd.type = b2_dynamicBody;
	    bd.position.Set(-42.0f+x_sreenivas, 39.0f+y_sreenivas);
	    b4 = m_world->CreateBody(&bd);
	    b4->CreateFixture(&shape, 10.0f);
      }
	
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-38.0f+x_sreenivas, 43.0f+y_sreenivas);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }

 //Two horizontal shelfs at big ball

    {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(-10.0f+x_srinath, 14.35f+y_srinath);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

  //The big ball central

    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 3;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 5.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(0.0f+x_srinath, 22.0f+y_srinath);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    } 

    //The pulley system left

    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-26+x_srinath,18+y_srinath);
      bd->fixedRotation = true;
      
      //The open box

      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 3.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 3.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 3.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //The bar left

      bd->position.Set(-6+x_srinath,15+y_srinath);	
      b2PolygonShape bs4;
      bs4.SetAsBox(0.3,6, b2Vec2(0.0f,0.0f), 1.56);
      fd1->shape = &bs4;
      fd1->density = 10.0;
      fd1->friction = 0.4;	  
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      // The pulley joint left

      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(-26+x_srinath, 15+y_srinath); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(0+x_srinath, 10+y_srinath); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-26+x_srinath, 20+y_srinath); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(-19+x_srinath, 15+y_srinath); // Anchor point for ground 2 in world axis
  
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);

    }  
   
   //bar below tray left
  
   {
     b2PolygonShape shape;
      shape.SetAsBox(2.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(-26+x_srinath,12+y_srinath);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);     
   }

//=================================================================-------------------------srinath right part

 //Two horizontal shelfs at big ball

    {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(10.0f+x_srinath, 14.34f+y_srinath);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

 //Top pendulum horizontal shelf right
 
   {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f ,0.25f);
	
      b2BodyDef bd;
      bd.position.Set(38.0f+x_sreenivas, 35.0f+y_sreenivas);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

 //The pendulum that knocks the ball right

    {
      b2Body* b2;
      {
	    b2PolygonShape shape;
	    shape.SetAsBox(1.5f, 0.25f);
	  
	    b2BodyDef bd;
	    bd.position.Set(38.0f+x_sreenivas, 43.0f+y_sreenivas);
	    b2 = m_world->CreateBody(&bd);
	    b2->CreateFixture(&shape, 10.0f);
      }
	
      b2Body* b4;
      {
	   b2PolygonShape shape;
	   shape.SetAsBox(1.0f, 1.0f);
	  
	   b2BodyDef bd;
	   bd.type = b2_dynamicBody;
	   bd.position.Set(42.0f+x_sreenivas, 39.0f+y_sreenivas);
	   b4 = m_world->CreateBody(&bd);
	   b4->CreateFixture(&shape, 10.0f);
      }
	
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(38.0f+x_sreenivas, 43.0f+y_sreenivas);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }  

    //The pulley system right

    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(26+x_srinath,18+y_srinath);
      bd->fixedRotation = true;
      
      //The open box

      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 3.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 3.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 3.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //The bar

      bd->position.Set(6+x_srinath,15+y_srinath);	
      b2PolygonShape bs4;
      bs4.SetAsBox(0.3,6, b2Vec2(0.0f,0.0f), -1.56);
      fd1->shape = &bs4;
      fd1->density = 10.0;
      fd1->friction = 0.4;	  
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      // The pulley joint right

      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(26+x_srinath, 15+y_srinath); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(0+x_srinath, 10+y_srinath); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(26+x_srinath, 20+y_srinath); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(19+x_srinath, 15+y_srinath); // Anchor point for ground 2 in world axis
  
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);

    }  

     //bar below tray right
  
   {
     b2PolygonShape shape;
      shape.SetAsBox(2.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(26+x_srinath,12+y_srinath);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);     
   }     

      //Wedge shape at bottom

    {
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(0.0f,0.0f), -0.63f);

      b2BodyDef bd;
      bd.position.Set(-2.0f+x_srinath, 9.4f+y_srinath);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
                  
      b2PolygonShape shape1;    // vertical bar
      shape1.SetAsBox(0.25f, 4.6f, b2Vec2(0.0f,0.0f), 0.0f);

      b2BodyDef bd1;
      bd1.position.Set(-7.5f+x_srinath, 9.2f+y_srinath);
      b2Body* ground1 = m_world->CreateBody(&bd1);
      ground1->CreateFixture(&shape1, 0.0f);

     
    }
    
      //Vertical bars to stop falling balls

    {
      b2PolygonShape shape;
      shape.SetAsBox(0.25f, 1.0f, b2Vec2(0.0f,0.0f), 0.0f);

      b2BodyDef bd;
      bd.position.Set(18.0f+x_srinath, 15.34f+y_srinath);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);

                      //left part
      b2BodyDef bd1;
      bd1.position.Set(-18.0f+x_srinath, 15.34f+y_srinath);
      b2Body* ground1 = m_world->CreateBody(&bd1);
      ground1->CreateFixture(&shape, 0.0f);

    }  
   
//=====================================================================================----------------------------srinath code ends

    }
  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}

