#ifndef SMMAP_H
#define SMMAP_H
#include <gmapping/grid/map.h>
#include <gmapping/grid/harray2d.h>
#include <gmapping/utils/point.h>
#define SIGHT_INC 1

namespace GMapping {

struct PointAccumulator{
	typedef point<float> FloatPoint;
	/* before 
	PointAccumulator(int i=-1): acc(0,0), n(0), visits(0){assert(i==-1);}
	*/
	/*after begin*/
	PointAccumulator(): acc(0,0), n(0), visits(0){}
	PointAccumulator(int i): acc(0,0), n(0), visits(0){assert(i==-1);}
	/*after end*/
        inline void update(bool value, const Point& p=Point(0,0), int detected=0);
        inline int getLabel();
	inline Point mean() const {return 1./n*Point(acc.x, acc.y);}
	inline operator double() const { return visits?(double)n*SIGHT_INC/(double)visits:-1; }
	inline void add(const PointAccumulator& p) {acc=acc+p.acc; n+=p.n; visits+=p.visits; }
	static const PointAccumulator& Unknown();
	static PointAccumulator* unknown_ptr;
	FloatPoint acc;
	int n, visits;
	inline double entropy() const;

	// Label entropy
	// This is used for semantic mapping, where we want to know the uncertainty
	// of the label assigned to a point.
	// The label is the detected value, which can be a class ID or a semantic label.
	int label_prob[12] = {0};
	int label_modifier = 6;
	int frees_modifier = 3;
};

void PointAccumulator::update(bool value, const Point& p, int detected){
	if (value) {
		// Assign Detected Value for Semantic Mapping
		if (detected > 0) {
			// Pergeseran value label objek
			for (int i = 0; i < 12 - label_modifier; i++){
				label_prob[i] = label_prob[i + label_modifier];
			}
			for (int i = 12 - label_modifier; i < 12; i++){
				label_prob[i] = detected;
			}
		} else {
			// Pergeseran value label free space
			for (int i = 0; i < 12 - frees_modifier; i++){
				label_prob[i] = label_prob[i + frees_modifier];
			}
			for (int i = 12 - frees_modifier; i < 12; i++){
				label_prob[i] = detected;
			}
		}
		acc.x+= static_cast<float>(p.x);
		acc.y+= static_cast<float>(p.y); 
		n++; 
		visits+=SIGHT_INC;
	} else
		visits++;
}



int PointAccumulator::getLabel() {
	// Calculate the mode of the label_prob array
	// This will be the label assigned to the point
	int mode = label_prob[0];
	int modeCount = 0;
	for (int i = 0; i < 12; i++) {
		int count = 0;
		for (int j = 0; j < 12; j++) {
			if(label_prob[j] == label_prob[i])
				count++;
		}
		if(count > modeCount) {
			modeCount = count;
			mode = label_prob[i];
		}
	}
	return mode;
}

double PointAccumulator::entropy() const{
	if (!visits)
		return -log(.5);
	if (n==visits || n==0)
		return 0;
	double x=(double)n*SIGHT_INC/(double)visits;
	return -( x*log(x)+ (1-x)*log(1-x) );
}


typedef Map<PointAccumulator,HierarchicalArray2D<PointAccumulator> > ScanMatcherMap;

};

#endif 
