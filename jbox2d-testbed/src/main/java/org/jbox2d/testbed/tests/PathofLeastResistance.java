package org.jbox2d.testbed.tests;

import java.util.ArrayList;
import java.util.Iterator;

import org.jbox2d.collision.shapes.CircleShape;
import org.jbox2d.collision.shapes.EdgeShape;
import org.jbox2d.collision.shapes.PolygonShape;
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
import org.jbox2d.testbed.framework.TestbedSettings;
import org.jbox2d.testbed.framework.TestbedTest;
import org.jbox2d.testbed.tests.SensorTest.BoolWrapper;

public class PathofLeastResistance extends TestbedTest{

	Body[] sensors = new Body[4];
	ArrayList<Body> flagforremoval = new ArrayList<Body>();
	int frame = 0;
	Body pivot;
	RevoluteJoint pusher;
	
	@Override
	public void initTest(boolean deserialized) {
		if(deserialized) return;
		
		PolygonShape shape = new PolygonShape();
		shape.setAsBox(2, 2);

		FixtureDef sd = new FixtureDef();
		sd.shape = shape;
		sd.friction = 0.3f;
		sd.density = .2f;
		
		BodyDef bd = new BodyDef();
		bd.type = BodyType.STATIC;
		bd.position = new Vec2(-15, 20);
		Body fulcrum = getWorld().createBody(bd);
		fulcrum.createFixture(sd);
		
		CircleShape cs = new CircleShape();

		cs.setRadius(2);
		sd.shape = cs;
		bd.position = new Vec2(-15,-5);
		bd.type = BodyType.KINEMATIC;
		pivot = getWorld().createBody(bd);
		pivot.createFixture(sd);
		
		shape.setAsBox(5, 1);
		sd.shape = shape;
		bd.type = BodyType.DYNAMIC;
		bd.position = new Vec2(-15, 0);
		bd.angle = (float)Math.PI/2;
		Body entity = getWorld().createBody(bd);
		entity.createFixture(sd);
		
		cs.setRadius(2);
		sd.shape = cs;
		bd.type = BodyType.DYNAMIC;
		bd.position = new Vec2(-20, 2);
		Body pusher = getWorld().createBody(bd);
		pusher.createFixture(sd);
		
		WeldJointDef wjd = new WeldJointDef();
		wjd.initialize(pivot, pusher, pivot.getWorldCenter());
		wjd.collideConnected = false;
		getWorld().createJoint(wjd);
		
		
		RevoluteJointDef rjd = new RevoluteJointDef();
		rjd.initialize(pivot, entity, pivot.getWorldCenter());
		rjd.collideConnected = false;
		getWorld().createJoint(rjd);
		
		DistanceJointDef djd = new DistanceJointDef();
		djd.frequencyHz = 2;
		djd.initialize(fulcrum, entity, fulcrum.getWorldCenter(), entity.getWorldCenter().add(new Vec2(0,5)));
		djd.collideConnected = false;
		getWorld().createJoint(djd);

		
		
		cs.setRadius(2);
		sd.shape = cs;
		bd.position = new Vec2(15,-5);
		bd.type = BodyType.STATIC;
		Body pivots = getWorld().createBody(bd);
		pivots.createFixture(sd);
		
		shape.setAsBox(5, 1);
		sd.shape = shape;
		bd.type = BodyType.DYNAMIC;
		bd.position = new Vec2(15, 0);
		bd.angle = (float)Math.PI/2;
		entity = getWorld().createBody(bd);
		entity.createFixture(sd);
		
		cs.setRadius(2);
		sd.shape = cs;
		bd.type = BodyType.DYNAMIC;
		bd.position = new Vec2(20, 2);
		pusher = getWorld().createBody(bd);
		pusher.createFixture(sd);
		
		wjd = new WeldJointDef();
		wjd.frequencyHz = 3;
		wjd.initialize(pivots, entity, pivots.getWorldCenter());
		wjd.collideConnected = false;
		getWorld().createJoint(wjd);
		
		
		rjd = new RevoluteJointDef();
		rjd.initialize(pivots, pusher, pivots.getWorldCenter());
		rjd.collideConnected = false;
		rjd.enableMotor = true;
		rjd.maxMotorTorque = pusher.getInertia()*(pusher.getWorldCenter().sub(pivots.getWorldCenter()).length())*40*60;
		rjd.motorSpeed = 2;
		this.pusher = (RevoluteJoint)getWorld().createJoint(rjd);
		
				
	}
	
	public void step(TestbedSettings settings) {
	    // TODO Auto-generated method stub
	    super.step(settings);
	    pivot.setAngularVelocity(3);
	}


	@Override
	public String getTestName() {
		// TODO Auto-generated method stub
		return "Path of Least Resistance";
	}

}
