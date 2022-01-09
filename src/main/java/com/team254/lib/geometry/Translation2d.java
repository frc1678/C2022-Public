package com.team254.lib.geometry;

import com.team254.lib.util.Util;

import java.text.DecimalFormat;

/**
 * A translation in a 2d coordinate frame. Translations are simply shifts in an (x, y) plane.
 */
public class Translation2d implements ITranslation2d<Translation2d> {
    protected static final Translation2d kIdentity = new Translation2d();

    public static final Translation2d identity() {
        return kIdentity;
    }

    protected double x_;
    protected double y_;

    public Translation2d() {
        x_ = 0;
        y_ = 0;
    }

    public Translation2d(double x, double y) {
        x_ = x;
        y_ = y;
    }

    public Translation2d(final Translation2d other) {
        x_ = other.x_;
        y_ = other.y_;
    }

    public Translation2d(final Translation2d start, final Translation2d end) {
        x_ = end.x_ - start.x_;
        y_ = end.y_ - start.y_;
    }
    
    public static Translation2d fromPolar(Rotation2d direction, double magnitude){
    	return new Translation2d(direction.cos() * magnitude, direction.sin() * magnitude);
    }

    /**
     * The "norm" of a transform is the Euclidean distance in x and y.
     *
     * @return sqrt(x ^ 2 + y ^ 2)
     */
    public double norm() {
        return Math.hypot(x_, y_);
    }

    public double norm2() {
        return x_ * x_ + y_ * y_;
    }
	
	/**
	 * Normalizing a vector scales it so that its norm is 1 while maintaining its direction.
	 * If input is a zero vector, return a zero vector.
	 * 
	 * @return r / norm(r) or (0,0)
	 */
	public Translation2d normalize() {
		if(epsilonEquals(identity(),Util.kEpsilon)) return this;
		return scale(1.0/norm());
	}

    public double x() {
        return x_;
    }

    public double y() {
        return y_;
    }
    
    public void setX(double x){
    	x_ = x;
    }
    
    public void setY(double y){
    	y_ = y;
    }
    

    /**
     * We can compose Translation2d's by adding together the x and y shifts.
     *
     * @param other The other translation to add.
     * @return The combined effect of translating by this object and the other.
     */
    public Translation2d translateBy(final Translation2d other) {
        return new Translation2d(x_ + other.x_, y_ + other.y_);
    }

    /**
     * We can also rotate Translation2d's. See: https://en.wikipedia.org/wiki/Rotation_matrix
     *
     * @param rotation The rotation to apply.
     * @return This translation rotated by rotation.
     */
    public Translation2d rotateBy(final Rotation2d rotation) {
        return new Translation2d(x_ * rotation.cos() - y_ * rotation.sin(), x_ * rotation.sin() + y_ * rotation.cos());
    }

    public Rotation2d direction() {
        return new Rotation2d(x_, y_, true);
    }

    /**
     * The inverse simply means a Translation2d that "undoes" this object.
     *
     * @return Translation by -x and -y.
     */
    public Translation2d inverse() {
        return new Translation2d(-x_, -y_);
    }

    @Override
    public Translation2d interpolate(final Translation2d other, double x) {
        if (x <= 0) {
            return new Translation2d(this);
        } else if (x >= 1) {
            return new Translation2d(other);
        }
        return extrapolate(other, x);
    }

    public Translation2d extrapolate(final Translation2d other, double x) {
        return new Translation2d(x * (other.x_ - x_) + x_, x * (other.y_ - y_) + y_);
    }

    public Translation2d scale(double s) {
        return new Translation2d(x_ * s, y_ * s);
    }

    public boolean epsilonEquals(final Translation2d other, double epsilon) {
        return Util.epsilonEquals(x(), other.x(), epsilon) && Util.epsilonEquals(y(), other.y(), epsilon);
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(x_) + "," + fmt.format(y_) + ")";
    }

    @Override
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return fmt.format(x_) + "," + fmt.format(y_);
    }

    public static double dot(final Translation2d a, final Translation2d b) {
        return a.x_ * b.x_ + a.y_ * b.y_;
    }
	
	/**
	 * The scalar projection of a vector u onto a vector v is the length of
	 * the "shadow" cast by u onto v under a "light" that is placed on a line
	 * normal to v and containing the endpoint of u, given that u and v share
	 * a starting point.
	 * tl;dr:
	 *    _*
	 * u  /|
	 *   / |
	 *  /  |       v
	 * *---+---------------->*
	 * \___/
	 *   |
	 *  scal_v(u)
	 * u.scal(v)
	 * 
	 * @return (u . v) / norm(v)
	 */
	public double scal(Translation2d v) {
		return dot(this, v) / v.norm();
	}
	/**
	 * The projection of a vector u onto a vector v is the vector in the direction
	 * of v with the magnitude u.scal(v).
	 * 
	 * @return u.scal(v) * v / norm(v)
	 */
	public Translation2d proj(Translation2d v) {
		return v.normalize().scale(scal(v));
	}
	/**
	 * https://stackoverflow.com/a/1167047/6627273
	 * A point D is considered "within" an angle ABC when
	 * cos(DBM) > cos(ABM)
	 * where M is the midpoint of AC, so ABM is half the angle ABC.
	 * The cosine of an angle can be computed as the dot product of two normalized
	 * vectors in the directions of its sides.
	 * Note that this definition of "within" does not include points that lie on
	 * the sides of the given angle.
	 * If `vertical` is true, then check not within the given angle, but within the
	 * image of that angle rotated by pi about its vertex.
	 * 
	 * @param Translation2d A
	 * 			A point on one side of the angle.
	 * @param Translation2d B
	 * 			The vertex of the angle.
	 * @param Translation2d C
	 * 			A point on the other side of the angle.
	 * @param boolean vertical
	 * 			Whether to check in the angle vertical to the one given
	 * @return Whether this translation is within the given angle.
	 * @author Joseph Reed
	 */
	public boolean isWithinAngle(Translation2d A, Translation2d B, Translation2d C, boolean vertical) {
		Translation2d M = A.interpolate(C,0.5); // midpoint
		Translation2d m = (new Translation2d(B,M)).normalize(); // mid-vector
		Translation2d a = (new Translation2d(B,A)).normalize(); // side vector
		Translation2d d = (new Translation2d(B,this)).normalize(); // vector to here
		if(vertical) {
			m = m.inverse();
			a = a.inverse();
		}
		return Translation2d.dot(d,m) > Translation2d.dot(a,m);
	}
	public boolean isWithinAngle(Translation2d A, Translation2d B, Translation2d C) {
		return isWithinAngle(A,B,C,false);
	}
	/** Assumes an angle centered at the origin. */
	public boolean isWithinAngle(Translation2d A, Translation2d C, boolean vertical) {
		return isWithinAngle(A,identity(),C,vertical);
	}
	public boolean isWithinAngle(Translation2d A, Translation2d C) {
		return isWithinAngle(A,C,false);
	}

    public static Rotation2d getAngle(final Translation2d a, final Translation2d b) {
        double cos_angle = dot(a, b) / (a.norm() * b.norm());
        if (Double.isNaN(cos_angle)) {
            return new Rotation2d();
        }
        return Rotation2d.fromRadians(Math.acos(Math.min(1.0, Math.max(cos_angle, -1.0))));
    }

    public static double cross(final Translation2d a, final Translation2d b) {
        return a.x_ * b.y_ - a.y_ * b.x_;
    }
	
	/**
	 * The distance between a point and a line can be computed as a scalar projection.
	 * 
	 * @param Translation2d a
	 * 			One point on the line.
	 * @param Translation2d b
	 * 			Another point on the line.
	 */
	public double distanceToLine(Translation2d a, Translation2d b) {
		Translation2d point = new Translation2d(a,this);
		Translation2d line = new Translation2d(a,b);
		Translation2d perpLine = line.rotateBy(new Rotation2d(90));
		return Math.abs(point.scal(perpLine));
		// let's use readable code from now on, not golfed one-liners, shall we?
		//return Math.abs((new Translation2d(a,this))scal((new Translation2d(a,b)).rotateBy(new Rotation2d(90))));
	}

    @Override
    public double distance(final Translation2d other) {
        return inverse().translateBy(other).norm();
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof Translation2d)) return false;
        return distance((Translation2d)other) < Util.kEpsilon;
    }

    @Override
    public Translation2d getTranslation() {
        return this;
    }
}
