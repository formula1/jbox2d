package org.jbox2d.testbed.tests;

import org.jbox2d.collision.shapes.ChainShape;
import org.jbox2d.collision.shapes.PolygonShape;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.BodyDef;
import org.jbox2d.dynamics.BodyType;
import org.jbox2d.dynamics.FixtureDef;
import org.jbox2d.particle.ParticleGroup;
import org.jbox2d.particle.ParticleGroupDef;
import org.jbox2d.particle.ParticleType;
import org.jbox2d.testbed.framework.TestbedTest;

public class DeadParticle extends TestbedTest{
	ParticleGroup pg;

	@Override
	public void initTest(boolean deserialized) {
		getWorld().setParticleRadius(.15f);
	      PolygonShape shape = new PolygonShape();
	      shape.setAsBox(5, 5);
	      ParticleGroupDef pd = new ParticleGroupDef();
	      pd.position.set(new Vec2(0, 0));
	      pd.flags = ParticleType.b2_springParticle;
	      pd.shape = shape;
	      pd.destroyAutomatically = true;
	      pg = getWorld().createParticleGroup(pd);
	      
	      BodyDef bd = new BodyDef();
	      FixtureDef fd = new FixtureDef();

	      bd.type = BodyType.STATIC;
	      bd.position = new Vec2(0,-10);
	      Body b = getWorld().createBody(bd);
	      
	      ChainShape floor = new ChainShape();
	      floor.createChain(new Vec2[]{new Vec2(-20,0), new Vec2(20,0)}, 2);
	      fd.shape = floor;
	      b.createFixture(fd);
	      
	      PolygonShape ps = new PolygonShape();
	      ps.setAsBox(2, 2);
	      fd.shape = ps;
	      fd.density = 5;
	      
	      bd.type = BodyType.DYNAMIC;
	      bd.position = new Vec2(0,20);
	      bd.bullet = true;
	      bd.setLinearVelocity(new Vec2(0,-400));
	      b = getWorld().createBody(bd);
	      b.createFixture(fd);
	}

	@Override
	public String getTestName() {
		// TODO Auto-generated method stub
		return "Dead Particle Bug";
	}

	
	
}
