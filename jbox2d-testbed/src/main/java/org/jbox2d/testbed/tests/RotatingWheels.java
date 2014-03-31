package org.jbox2d.testbed.tests;

import java.util.ArrayList;
import java.util.Iterator;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.Fixture;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.dynamics.contacts.Contact;
import org.jbox2d.dynamics.joints.DistanceJointDef;
import org.jbox2d.dynamics.joints.RevoluteJoint;
import org.jbox2d.dynamics.joints.RevoluteJointDef;
import org.jbox2d.dynamics.joints.WeldJointDef;
import org.jbox2d.dynamics.joints.WheelJoint;
import org.jbox2d.dynamics.joints.WheelJointDef;
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;
import org.jbox2d.testbed.tests.SensorTest.BoolWrapper;

public class RotatingWheels extends TestbedTest{

	WheelJoint pusher;
	Body left;
	Body right;
	WheelJoint orig;
	@Override
	public void initTest(boolean deserialized) {
		if(deserialized) return;
		
		EdgeShape es = new EdgeShape();
		es.set(new Vec2(-20,0), new Vec2(20,0));

		FixtureDef sd = new FixtureDef();
		sd.shape = es;
		sd.friction = 0.3f;
		sd.density = .2f;
		
		BodyDef bd = new BodyDef();
		bd.type = BodyType.STATIC;
		bd.position = new Vec2(0, -5);
		Body gr = getWorld().createBody(bd);
		gr.createFixture(sd);

		
		
		
		
		CircleShape cs = new CircleShape();
		cs.setRadius(2);
		
		sd.shape = cs;
		bd.position = new Vec2(-10,7);
		bd.gravityScale = 0;
		bd.type = BodyType.DYNAMIC;
		left = getWorld().createBody(bd);
		left.createFixture(sd);
		
		sd.shape = cs;
		bd.position = new Vec2(10,7);
		bd.type = BodyType.DYNAMIC;
		right = getWorld().createBody(bd);
		right.createFixture(sd);
		
		Vec2 diff = right.getWorldCenter().sub(left.getPosition());
		float diffang = (float)Math.atan2(diff.y, diff.x);
		diffang -= right.getAngle();
		Vec2 normal = new Vec2((float)Math.cos(diffang), (float)Math.sin(diffang));

		Vec2 mid = left.getWorldCenter().add(right.getWorldCenter()).mul(.5f);
//		wj.initialize(left, right, left.getWorldCenter().add(right.getWorldCenter()).mul(.5f), normal);
		WheelJointDef wj = new WheelJointDef();
		wj.bodyA = left;
		wj.bodyB = right;
		wj.localAxisA.set(normal); 
		wj.localAnchorA.set(mid.sub(left.getWorldCenter()));
		wj.localAnchorB.set(mid.sub(right.getWorldCenter()));
		wj.enableMotor = true;
		wj.maxMotorTorque = (left.getInertia()*right.getInertia())*MathUtils.PI*2*60;
		wj.motorSpeed = 2;
		wj.frequencyHz = 3;
		wj.dampingRatio = 1;
		orig = (WheelJoint)getWorld().createJoint(wj);
		
	}
	
	public void step(TestbedSettings settings) {
	    // TODO Auto-generated method stub
	    super.step(settings);
	    float k = .5f;
	    float damp = .1f;
	    if(orig.getJointSpeed()+orig.getJointAngle() != 0)
	    	orig.setMotorSpeed(-1f*((1-damp*k)*orig.getJointSpeed()+k*orig.getJointAngle()));
/*
 * Frequency is applying force every moment
 * -this means accelleration
 * 
 * dampening is the amount of speed that is considered to avoid a loop
 * -if no speed is considered then the it will go infinite
 * 
 * dampening's only purpose is to prevent infinite. as such it will get the difference 
 * 
 */
//	    left.applyForceToCenter(new Vec2(20,20).mul(left.getMass()));
//	    right.applyForceToCenter(new Vec2(-20,20).mul(right.getMass()));
	}


	@Override
	public String getTestName() {
		// TODO Auto-generated method stub
		return "Rotating Wheel";
	}

}
