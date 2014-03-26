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
		bd.position = new Vec2(-3,0);
		bd.type = BodyType.DYNAMIC;
		Body b1 = getWorld().createBody(bd);
		b1.createFixture(sd);

		sd.shape = cs;
		bd.position = new Vec2(3,0);
		bd.type = BodyType.DYNAMIC;
		Body b2 = getWorld().createBody(bd);
		b2.createFixture(sd);
		
		WheelJointDef wj = new WheelJointDef();
		wj.frequencyHz = 2;
		wj.dampingRatio =1f;
		wj.initialize(b1, b2, b1.getWorldCenter().add(b2.getWorldCenter()).mul(.5f), new Vec2(1,0));
		pusher = (WheelJoint)getWorld().createJoint(wj);
		pusher.enableMotor(true);
		pusher.setMaxMotorTorque(b1.getInertia()+b2.getInertia());
		pusher.setMotorSpeed(10);
		
		sd.shape = cs;
		bd.position = new Vec2(-10,7);
		bd.gravityScale = 0;
		bd.type = BodyType.STATIC;
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

		
		wj.initialize(left, right, left.getWorldCenter().add(right.getWorldCenter()).mul(.5f), normal);
		wj.enableMotor = true;
		wj.maxMotorTorque = (left.getMass()*right.getMass())*MathUtils.PI*2*60;
		wj.motorSpeed = 2;
		getWorld().createJoint(wj);
		
	}
	
	public void step(TestbedSettings settings) {
	    // TODO Auto-generated method stub
	    super.step(settings);
	    
	    left.applyForceToCenter(new Vec2(20,20).mul(left.getMass()));
	    right.applyForceToCenter(new Vec2(-20,20).mul(right.getMass()));
	}


	@Override
	public String getTestName() {
		// TODO Auto-generated method stub
		return "Rotating Wheel";
	}

}
