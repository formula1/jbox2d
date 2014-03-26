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

public class LinearResistance extends TestbedTest{

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
		
		shape = new PolygonShape();
		shape.setAsBox(2, 2);

		sd = new FixtureDef();
		sd.shape = shape;
		sd.friction = 0.3f;
		sd.density = .2f;
		
		bd.type = BodyType.STATIC;
		bd.position = new Vec2(15, 20);
		Body fulcrum2 = getWorld().createBody(bd);
		fulcrum2.createFixture(sd);
		
				
		shape.setAsBox(2, 2);
		sd.shape = shape;
		bd.type = BodyType.DYNAMIC;
		bd.position = new Vec2(-15, 0);
		bd.angle = (float)Math.PI/2;
		Body entity = getWorld().createBody(bd);
		entity.createFixture(sd);
		
		DistanceJointDef djd = new DistanceJointDef();
		djd.frequencyHz = .5f;
		djd.initialize(fulcrum, entity, fulcrum.getWorldCenter(), entity.getWorldCenter());
		djd.collideConnected = true;
		getWorld().createJoint(djd);


		shape.setAsBox(2, 2);
		sd.shape = shape;
		bd.type = BodyType.DYNAMIC;
		bd.position = new Vec2(15, 0);
		bd.angle = (float)Math.PI/2;
		entity = getWorld().createBody(bd);
		entity.createFixture(sd);
		
		WeldJointDef wjd = new WeldJointDef();
		wjd.frequencyHz = .5f;
		wjd.initialize(fulcrum2, entity, fulcrum2.getWorldCenter());
		wjd.collideConnected = true;
		getWorld().createJoint(wjd);

		FixtureDef sensor = new FixtureDef();
		sensor.isSensor = true;
		shape.setAsBox(2, 20);
		sensor.shape = shape;
		
		BodyDef bds = new BodyDef();
		
		for(int i = 0; i<4;i++){
			bds.angle = i*(float)Math.PI/2;
			bds.position = new Vec2((float)Math.cos(bds.angle), (float)Math.sin(bds.angle)).mul(20);
			sensors[i] = getWorld().createBody(bds);
		}
		
	}
	
	public void beginContact(Contact contact) {
		Fixture fixtureA = contact.getFixtureA();
		Fixture fixtureB = contact.getFixtureB();
		for(int i=0;i<sensors.length;i++){
			if (fixtureA.getBody() == sensors[i])
				flagforremoval.add(fixtureB.getBody());
			else if(fixtureB.getBody() == sensors[i])
				flagforremoval.add(fixtureA.getBody());
		}
	}
	public void step(TestbedSettings settings) {
	    // TODO Auto-generated method stub
	    super.step(settings);

	    Iterator<Body> i = flagforremoval.iterator();
	    while(i.hasNext()){
	    	Body b = i.next();
	    	i.remove();
	    	getWorld().destroyBody(b);
	    }
	    frame = (frame+1)%120;
	    if(frame == 0){
			CircleShape cs = new CircleShape();
			cs.setRadius(2);
			FixtureDef sd = new FixtureDef();
			sd.shape = cs;
			sd.friction = 0.3f;
			sd.density = 40;
			
			BodyDef bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;
			bd.bullet = true;
			bd.position = new Vec2(15, -15);
			bd.setLinearVelocity(new Vec2(0,150));
			Body b = getWorld().createBody(bd);
			b.createFixture(sd);
			
			bd = new BodyDef();
			bd.type = BodyType.DYNAMIC;
			bd.bullet = true;
			bd.position = new Vec2(-15, -15);
			bd.setLinearVelocity(new Vec2(0,150));
			b = getWorld().createBody(bd);
			b.createFixture(sd);

	    }
	}


	@Override
	public String getTestName() {
		// TODO Auto-generated method stub
		return "Linear Resistance";
	}

}
