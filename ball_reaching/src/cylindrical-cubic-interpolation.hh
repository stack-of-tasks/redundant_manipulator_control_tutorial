// 
// Copyright (C) 2012 LAAS-CNRS
//
// Author: Florent Lamiraux
//

#ifndef SOT_REACHING_CYLINDRIC_CUBIC_INTERPOLATION_HH
# define SOT_REACHING_CYLINDRIC_CUBIC_INTERPOLATION_HH

# include "cubic-interpolation.hh"

namespace dynamicgraph {
  namespace sot {
    namespace reaching {
      class CylindricalCubicInterpolationSE3 : public CubicInterpolationSE3
      {
	DYNAMIC_GRAPH_ENTITY_DECL();
      public:
	virtual ~CylindricalCubicInterpolationSE3 ();
	CylindricalCubicInterpolationSE3 (const std::string& name);
	/// Documentation
	virtual std::string getDocString () const;
	/// Start tracking
	virtual void doStart (const double& duration);
      private:
	dynamicgraph::SignalPtr < MatrixHomogeneous, int > localFrameSIN_;
	
	MatrixHomogeneous& computeReference (MatrixHomogeneous& reference,
					     const int& time);

	MatrixHomogeneous initialChestPose_;
	MatrixHomogeneous inv_;
	MatrixHomogeneous localPose_;
	maal::boost::Vector cylindricalCoord_;
	maal::boost::Vector globalPos_;

      }; // class CylindricalCubicInterpolationSE3
    } // reaching
  } // namespace sot
} // namespace dynamicgraph

#endif // SOT_REACHING_CYLINDRIC_CUBIC_INTERPOLATION_HH
