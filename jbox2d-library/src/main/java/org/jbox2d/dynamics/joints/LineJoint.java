package org.jbox2d.dynamics.joints;

/**
 * Created at 9:06:02 PM Jan 21, 2011
 */

import org.jbox2d.common.Mat22;
import org.jbox2d.common.Mat33;
import org.jbox2d.common.MathUtils;
import org.jbox2d.common.Rot;
import org.jbox2d.common.Settings;
import org.jbox2d.common.Transform;
import org.jbox2d.common.Vec2;
import org.jbox2d.dynamics.Body;
import org.jbox2d.dynamics.SolverData;
import org.jbox2d.dynamics.TimeStep;
import org.jbox2d.dynamics.World;
import org.jbox2d.dynamics.joints.Joint;
import org.jbox2d.dynamics.joints.LimitState;
import org.jbox2d.pooling.IWorldPool;

/**
 * @author Daniel Murphy
 */
public class LineJoint extends Joint {
        
        private final Vec2 m_localAnchor1 = new Vec2();
        private final Vec2 m_localAnchor2 = new Vec2();
        private final Vec2 m_localXAxis1 = new Vec2();
        private final Vec2 m_localYAxis1 = new Vec2();
        
        private final Vec2 m_axis = new Vec2();
        private final Vec2 m_perp = new Vec2();
        private float m_s1, m_s2;
        private float m_a1, m_a2;
        
        private final Mat22 m_K = new Mat22();
        private final Vec2 m_impulse = new Vec2();
        
        private float m_motorMass; // effective mass for motor/limit translational
                                                                // constraint.
        private float m_motorImpulse;
        
        private float m_lowerTranslation;
        private float m_upperTranslation;
        private float m_maxMotorForce;
        private float m_motorSpeed;
        
        private boolean m_enableLimit;
        private boolean m_enableMotor;
        private LimitState m_limitState;
        
        // Solver temp
        private int m_indexA;
        private int m_indexB;
        private float m_invMassA;
        private float m_invMassB;
        private float m_invIA;
        private float m_invIB;

        
        
        public Vec2 m_localCenterA, m_localCenterB;
        
        protected LineJoint(IWorldPool argPool, LineJointDef def) {
                super(argPool, def);
                m_localCenterA = new Vec2();
                m_localCenterB = new Vec2();
                m_localAnchor1.set(def.localAnchorA);
                m_localAnchor2.set(def.localAnchorB);
                m_localXAxis1.set(def.localAxisA);
                Vec2.crossToOut(1.0f, m_localXAxis1, m_localYAxis1);
                
                m_impulse.setZero();
                m_motorMass = 0.0f;
                m_motorImpulse = 0.0f;
                
                m_lowerTranslation = def.lowerTranslation;
                m_upperTranslation = def.upperTranslation;
                m_maxMotorForce = def.maxMotorForce;
                m_motorSpeed = def.motorSpeed;
                m_enableLimit = def.enableLimit;
                m_enableMotor = def.enableMotor;
                m_limitState = LimitState.INACTIVE;
                
                m_axis.setZero();
                m_perp.setZero();
        }
        
        /**
         * @see org.jbox2d.dynamics.joints.Joint#getAnchorA(org.jbox2d.common.Vec2)
         */
        @Override
        public void getAnchorA(Vec2 argOut) {
                m_bodyA.getWorldPointToOut(m_localAnchor1, argOut);
        }
        
        /**
         * @see org.jbox2d.dynamics.joints.Joint#getAnchorB(org.jbox2d.common.Vec2)
         */
        @Override
        public void getAnchorB(Vec2 argOut) {
                m_bodyB.getWorldPointToOut(m_localAnchor2, argOut);
                
        }
        
        /**
         * @see org.jbox2d.dynamics.joints.Joint#getReactionForce(float,
         *      org.jbox2d.common.Vec2)
         */
        @Override
        public void getReactionForce(float inv_dt, Vec2 argOut) {
                final Vec2 temp = pool.popVec2();
                temp.set(m_perp).mulLocal(m_impulse.x);
                argOut.set(m_axis).mulLocal(m_motorImpulse + m_impulse.y).addLocal(temp).mulLocal(inv_dt);
                pool.pushVec2(1);
        }
        
        /**
         * @see org.jbox2d.dynamics.joints.Joint#getReactionTorque(float)
         */
        @Override
        public float getReactionTorque(float inv_dt) {
                return 0.0f;
        }
        
        public float getJointTranslation() {
                Body b1 = m_bodyA;
                Body b2 = m_bodyB;
                
                Vec2 p1 = pool.popVec2();
                Vec2 p2 = pool.popVec2();
                Vec2 axis = pool.popVec2();
                b1.getWorldPointToOut(m_localAnchor1, p1);
                b2.getWorldPointToOut(m_localAnchor1, p2);
                p2.subLocal(p1);
                b1.getWorldVectorToOut(m_localXAxis1, axis);
                
                float translation = Vec2.dot(p2, axis);
                pool.pushVec2(3);
                return translation;
        }
        
        public float getJointSpeed() {
                Body b1 = m_bodyA;
                Body b2 = m_bodyB;
                
                final Vec2 r1 = pool.popVec2();
                final Vec2 r2 = pool.popVec2();
                final Vec2 p1 = pool.popVec2();
                final Vec2 p2 = pool.popVec2();
                
                r1.set(m_localAnchor1).subLocal(b1.getLocalCenter());
                r2.set(m_localAnchor2).subLocal(b2.getLocalCenter());
                Rot.mulToOut(b1.getTransform().q, r1, r1);
                Rot.mulToOut(b2.getTransform().q, r2, r2);
                
                p1.set(b1.m_sweep.c).addLocal(r1);
                p2.set(b2.m_sweep.c).addLocal(r2);
                p2.subLocal(p1);
                
                final Vec2 axis = pool.popVec2();
                b1.getWorldPointToOut(m_localXAxis1, axis);
                
                final Vec2 v1 = b1.m_linearVelocity;
                final Vec2 v2 = b2.m_linearVelocity;
                float w1 = b1.m_angularVelocity;
                float w2 = b2.m_angularVelocity;
                
                final Vec2 temp1 = pool.popVec2();
                final Vec2 temp2 = pool.popVec2();
                
                Vec2.crossToOut(w1, r1, temp1);
                Vec2.crossToOut(w2, r2, temp2);
                temp2.addLocal(v2).subLocal(v1).subLocal(temp1);
                float s2 = Vec2.dot(axis, temp2);
                
                Vec2.crossToOut(w1, axis, temp1);
                float speed = Vec2.dot(p2, temp1) + s2;
                
                pool.pushVec2(7);
                return speed;
        }
        
        public boolean isLimitEnabled() {
                return m_enableLimit;
        }
        
        public void enableLimit(boolean flag) {
                m_bodyA.setAwake(true);
                m_bodyB.setAwake(true);
                m_enableLimit = flag;
        }
        
        public float getLowerLimit() {
                return m_lowerTranslation;
        }
        
        public float getUpperLimit() {
                return m_upperTranslation;
        }
        
        public void setLimits(float lower, float upper) {
                assert (lower <= upper);
                m_bodyA.setAwake(true);
                m_bodyB.setAwake(true);
                m_lowerTranslation = lower;
                m_upperTranslation = upper;
        }
        
        public boolean isMotorEnabled() {
                return m_enableMotor;
        }
        
        public void enableMotor(boolean flag) {
                m_bodyA.setAwake(true);
                m_bodyB.setAwake(true);
                m_enableMotor = flag;
        }
        
        public void setMotorSpeed(float speed) {
                m_bodyA.setAwake(true);
                m_bodyB.setAwake(true);
                m_motorSpeed = speed;
        }
        
        public float getMotorSpeed(){
        	return m_motorSpeed;
        }
        
        public void setMaxMotorForce(float force) {
                m_bodyA.setAwake(true);
                m_bodyB.setAwake(true);
                m_maxMotorForce = force;
        }
        public float getMotorForce(float inv_dt) {
            return m_motorImpulse * inv_dt;
          }
        
        public float getMotorForce() {
                return m_motorImpulse;
        }
        

		@Override
		public void initVelocityConstraints(SolverData data) {
            m_indexA = m_bodyA.m_islandIndex;
            m_indexB = m_bodyB.m_islandIndex;
            m_localCenterA.set(m_bodyA.getLocalCenter());
            m_localCenterB.set(m_bodyB.getLocalCenter());
            m_invMassA = m_bodyA.m_invMass;
            m_invMassB = m_bodyB.m_invMass;
            m_invIA = m_bodyA.m_invI;
            m_invIB = m_bodyB.m_invI;
            
            Vec2 cA = data.positions[m_indexA].c;
            float aA = data.positions[m_indexA].a;
            Vec2 vA = data.velocities[m_indexA].v;
            float wA = data.velocities[m_indexA].w;

            Vec2 cB = data.positions[m_indexB].c;
            float aB = data.positions[m_indexB].a;
            Vec2 vB = data.velocities[m_indexB].v;
            float wB = data.velocities[m_indexB].w;
            
            // Compute the effective masses.
            final Rot qA = pool.popRot();
            final Rot qB = pool.popRot();
            final Vec2 r1 = pool.popVec2();
            final Vec2 r2 = pool.popVec2();
            final Vec2 temp = pool.popVec2();
            
            qA.set(aA);
            qB.set(aB);
            
            r1.set(m_localAnchor1).subLocal(m_localCenterA);
            r2.set(m_localAnchor2).subLocal(m_localCenterB);
            Rot.mulToOut(qA, r1, r1);
            Rot.mulToOut(qB, r2, r2);
            
            final Vec2 d = pool.popVec2();
            d.set(m_bodyB.m_sweep.c).addLocal(r2).subLocal(m_bodyA.m_sweep.c).subLocal(r1);
            
            
            // Compute motor Jacobian and effective mass.
            {
                    Rot.mulToOut(qA, m_localXAxis1, m_axis);
                    temp.set(d).addLocal(r1);
                    m_a1 = Vec2.cross(temp, m_axis);
                    m_a2 = Vec2.cross(r2, m_axis);
                    
                    m_motorMass = m_invMassA + m_invMassB + m_invIA * m_a1 * m_a1 + m_invIB * m_a2 * m_a2;
                    if (m_motorMass > Settings.EPSILON) {
                            m_motorMass = 1.0f / m_motorMass;
                    }
                    else {
                            m_motorMass = 0.0f;
                    }
            }
            
            // Prismatic constraint.
            {
                    Rot.mulToOut(qA, m_localYAxis1, m_perp);
                    
                    temp.set(d).addLocal(r1);
                    m_s1 = Vec2.cross(temp, m_perp);
                    m_s2 = Vec2.cross(r2, m_perp);
                    
                    float m1 = m_invMassA, m2 = m_invMassB;
                    float i1 = m_invIA, i2 = m_invIB;
                    
                    float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
                    float k12 = i1 * m_s1 * m_a1 + i2 * m_s2 * m_a2;
                    float k22 = m1 + m2 + i1 * m_a1 * m_a1 + i2 * m_a2 * m_a2;
                    
                    if(k22 == 0) k22 = 1.0f;
                    
                    m_K.ex.set(k11, k12);
                    m_K.ey.set(k12, k22);
            }
            
            // Compute motor and limit terms.
            if (m_enableLimit) {
                    float jointTranslation = Vec2.dot(m_axis, d);
                    if (MathUtils.abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.linearSlop) {
                            m_limitState = LimitState.EQUAL;
                    }
                    else if (jointTranslation <= m_lowerTranslation) {
                            if (m_limitState != LimitState.AT_LOWER) {
                                    m_limitState = LimitState.AT_LOWER;
                                    m_impulse.y = 0.0f;
                            }
                    }
                    else if (jointTranslation >= m_upperTranslation) {
                            if (m_limitState != LimitState.AT_UPPER) {
                                    m_limitState = LimitState.AT_UPPER;
                                    m_impulse.y = 0.0f;
                            }
                    }
                    else {
                            m_limitState = LimitState.INACTIVE;
                            m_impulse.y = 0.0f;
                    }
            }
            else {
                    m_limitState = LimitState.INACTIVE;
            }
            
            if (m_enableMotor == false) {
                    m_motorImpulse = 0.0f;
            }
            
            if (data.step.warmStarting) {
                    // Account for variable time step.
                    m_impulse.mulLocal(data.step.dtRatio);
                    m_motorImpulse *= data.step.dtRatio;
                    
                    final Vec2 P = pool.popVec2();
                    temp.set(m_axis).mulLocal(m_motorImpulse + m_impulse.y);
                    P.set(m_perp).mulLocal(m_impulse.x).addLocal(temp);
                    
                    float L1 = m_impulse.x * m_s1 + (m_motorImpulse + m_impulse.y) * m_a1;
                    float L2 = m_impulse.x * m_s2 + (m_motorImpulse + m_impulse.y) * m_a2;
                    
                    data.velocities[m_indexA].v.x -= m_invMassA * P.x;
                    data.velocities[m_indexA].v.y -= m_invMassA * P.y;
                    data.velocities[m_indexA].w -= m_invIA * L1;
                    
                    
                    data.velocities[m_indexB].v.x += m_invMassB * P.x;
                    data.velocities[m_indexA].v.y += m_invMassB * P.y;
                    data.velocities[m_indexB].w += m_invIB * L1;

                    pool.pushVec2(1);
            }
            else {
                    m_impulse.setZero();
                    m_motorImpulse = 0.0f;
            }

            
            pool.pushVec2(4);
			
		}

		@Override
		public void solveVelocityConstraints(SolverData data) {
            Vec2 vA = data.velocities[m_indexA].v;
            float wA = data.velocities[m_indexA].w;
            Vec2 vB = data.velocities[m_indexB].v;
            float wB = data.velocities[m_indexB].w;
            
            final Vec2 temp = pool.popVec2();
            
            // Solve linear motor constraint.
            if (m_enableMotor && m_limitState != LimitState.EQUAL) {
                    temp.set(vB).subLocal(vA);
                    float Cdot = Vec2.dot(m_axis, temp) + m_a2 * wB - m_a1 * wA;
                    float impulse = m_motorMass * (m_motorSpeed - Cdot);
                    float oldImpulse = m_motorImpulse;
                    float maxImpulse = data.step.dt * m_maxMotorForce;
                    m_motorImpulse = MathUtils.clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
                    impulse = m_motorImpulse - oldImpulse;
                    
                    final Vec2 P = pool.popVec2();
                    P.set(m_axis).mulLocal(impulse);
                    float L1 = impulse * m_a1;
                    float L2 = impulse * m_a2;
                    
                    temp.set(P).mulLocal(m_invMassA);
                    vA.subLocal(temp);
                    wA -= m_invIA * L1;
                    
                    temp.set(P).mulLocal(m_invMassB);
                    vB.addLocal(temp);
                    wB += m_invIB * L2;
                    pool.pushVec2(1);
            }
            
            temp.set(vB).subLocal(vA);
            float Cdot1 = Vec2.dot(m_perp, temp) + m_s2 * wB - m_s1 * wA;
            
            if (m_enableLimit && m_limitState != LimitState.INACTIVE) {
                    // Solve prismatic and limit constraint in block form.
                    temp.set(vB).subLocal(vA);
                    float Cdot2 = Vec2.dot(m_axis, temp) + m_a2 * wB - m_a1 * wA;
                    
                    final Vec2 Cdot = pool.popVec2();
                    Cdot.set(Cdot1, Cdot2);
                    
                    final Vec2 f1 = pool.popVec2();
                    f1.set(m_impulse);
                    final Vec2 df = pool.popVec2();
                    m_K.solveToOut(Cdot.negateLocal(), df); // just leave negated
                    m_impulse.addLocal(df);
                    
                    if (m_limitState == LimitState.AT_LOWER) {
                            m_impulse.y = MathUtils.max(m_impulse.y, 0.0f);
                    }
                    else if (m_limitState == LimitState.AT_UPPER) {
                            m_impulse.y = MathUtils.min(m_impulse.y, 0.0f);
                    }
                    
                    // f2(1) = invK(1,1) * (-Cdot(1) - K(1,2) * (f2(2) - f1(2))) + f1(1)
                    float b = -Cdot1 - (m_impulse.y - f1.y) * m_K.ey.x;
                    float f2r;
                    if (m_K.ex.x != 0.0f) {
                            f2r = b / m_K.ex.x + f1.x;
                    }
                    else {
                            f2r = f1.x;
                    }
                    
                    m_impulse.x = f2r;
                    
                    df.set(m_impulse).subLocal(f1);
                    
                    final Vec2 P = pool.popVec2();
                    temp.set(m_axis).mulLocal(df.y);
                    P.set(m_perp).mulLocal(df.x).addLocal(temp);
                    
                    float L1 = df.x * m_s1 + df.y * m_a1;
                    float L2 = df.x * m_s2 + df.y * m_a2;
                    
                    temp.set(P).mulLocal(m_invMassA);
                    vA.subLocal(temp);
                    wA -= m_invIA * L1;
                    
                    temp.set(P).mulLocal(m_invMassB);
                    vB.addLocal(temp);
                    wB += m_invIB * L2;
                    pool.pushVec2(4);
            }
            else {
                    // Limit is inactive, just solve the prismatic constraint in block
                    // form.
                    float df;
                    if (m_K.ex.x != 0.0f) {
                            df = -Cdot1 / m_K.ex.x;
                    }
                    else {
                            df = 0.0f;
                    }
                    m_impulse.x += df;
                    
                    final Vec2 P = pool.popVec2();
                    P.set(m_perp).mulLocal(df);
                    
                    float L1 = df * m_s1;
                    float L2 = df * m_s2;
                    
                    temp.set(P).mulLocal(m_invMassA);
                    vA.subLocal(temp);
                    wA -= m_invIA * L1;
                    
                    temp.set(P).mulLocal(m_invMassB);
                    vB.addLocal(temp);
                    wB += m_invIB * L2;
                    pool.pushVec2(1);
            }
            
            pool.pushVec2(1);
            
            data.velocities[m_indexA].w = wA;
            data.velocities[m_indexB].w = wB;			
		}

		@Override
		public boolean solvePositionConstraints(SolverData data) {
            
            final Vec2 c1 = data.positions[m_indexA].c;
            float a1 = data.positions[m_indexA].a;
            
            final Vec2 c2 = data.positions[m_indexB].c;
            float a2 = data.positions[m_indexA].a;
            
            // Solve linear limit constraint.
            float linearError = 0.0f, angularError = 0.0f;
            boolean active = false;
            float C2 = 0.0f;
            
            Mat22 R1 = pool.popMat22();
            Mat22 R2 = pool.popMat22();
            R1.set(a1);
            R2.set(a2);
            
            final Vec2 r1 = pool.popVec2();
            final Vec2 r2 = pool.popVec2();
            final Vec2 temp = pool.popVec2();
            final Vec2 d = pool.popVec2();
            
            r1.set(m_localAnchor1).subLocal(m_localCenterA);
            r2.set(m_localAnchor2).subLocal(m_localCenterB);
            Mat22.mulToOut(R1, r1, r1);
            Mat22.mulToOut(R2, r2, r2);
            d.set(c2).addLocal(r2).subLocal(c1).subLocal(r1);
            
            if (m_enableLimit) {
                    Mat22.mulToOut(R1, m_localXAxis1, m_axis);
                    
                    temp.set(d).addLocal(r1);
                    m_a1 = Vec2.cross(temp, m_axis);
                    m_a2 = Vec2.cross(r2, m_axis);
                    
                    float translation = Vec2.dot(m_axis, d);
                    if (MathUtils.abs(m_upperTranslation - m_lowerTranslation) < 2.0f * Settings.linearSlop) {
                            // Prevent large angular corrections
                            C2 = MathUtils.clamp(translation, -Settings.maxLinearCorrection, Settings.maxLinearCorrection);
                            linearError = MathUtils.abs(translation);
                            active = true;
                    }
                    else if (translation <= m_lowerTranslation) {
                            // Prevent large linear corrections and allow some slop.
                            C2 = MathUtils.clamp(translation - m_lowerTranslation + Settings.linearSlop,
                                            -Settings.maxLinearCorrection, 0.0f);
                            linearError = m_lowerTranslation - translation;
                            active = true;
                    }
                    else if (translation >= m_upperTranslation) {
                            // Prevent large linear corrections and allow some slop.
                            C2 = MathUtils.clamp(translation - m_upperTranslation - Settings.linearSlop, 0.0f,
                                            Settings.maxLinearCorrection);
                            linearError = translation - m_upperTranslation;
                            active = true;
                    }
            }
            
            Mat22.mulToOut(R1, m_localYAxis1, m_perp);
            
            temp.set(d).addLocal(r1);
            m_s1 = Vec2.cross(temp, m_perp);
            m_s2 = Vec2.cross(r2, m_perp);
            
            final Vec2 impulse = pool.popVec2();
            float C1;
            C1 = Vec2.dot(m_perp, d);
            
            linearError = MathUtils.max(linearError, MathUtils.abs(C1));
            angularError = 0.0f;
            
            if (active) {
                    float m1 = m_invMassA, m2 = m_invMassB;
                    float i1 = m_invIA, i2 = m_invIB;
                    
                    float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
                    float k12 = i1 * m_s1 * m_a1 + i2 * m_s2 * m_a2;
                    float k22 = m1 + m2 + i1 * m_a1 * m_a1 + i2 * m_a2 * m_a2;
                    
                    m_K.ex.set(k11, k12);
                    m_K.ey.set(k12, k22);
                    
                    final Vec2 C = pool.popVec2();
                    C.x = C1;
                    C.y = C2;
                    
                    m_K.solveToOut(C.negateLocal(), impulse);
                    pool.pushVec2(1);
            }
            else {
                    float m1 = m_invMassA, m2 = m_invMassB;
                    float i1 = m_invIA, i2 = m_invIB;
                    
                    float k11 = m1 + m2 + i1 * m_s1 * m_s1 + i2 * m_s2 * m_s2;
                    
                    float impulse1;
                    if (k11 != 0.0f) {
                            impulse1 = -C1 / k11;
                    }
                    else {
                            impulse1 = 0.0f;
                    }
                    
                    impulse.x = impulse1;
                    impulse.y = 0.0f;
            }
            
            final Vec2 P = pool.popVec2();
            temp.set(m_axis).mulLocal(impulse.y);
            P.set(m_perp).mulLocal(impulse.x).add(temp);
            
            float L1 = impulse.x * m_s1 + impulse.y * m_a1;
            float L2 = impulse.x * m_s2 + impulse.y * m_a2;
            
            temp.set(P).mulLocal(m_invMassA);
            c1.subLocal(temp);
            a1 -= m_invIA * L1;
            
            temp.set(P).mulLocal(m_invMassB);
            c2.addLocal(temp);
            a2 += m_invIB * L2;
            
            // TODO_ERIN remove need for this.
            data.positions[m_indexA].a = a1;
            data.positions[m_indexB].a = a2;
            
            pool.pushVec2(6);
            pool.pushMat22(2);
            
            return linearError <= Settings.linearSlop && angularError <= Settings.angularSlop;
		}
}
