#pragma once

#include "Section.h"

class Tube :  public Part
{
friend class CTRFactory;

std::vector<Section> sections;
double kxy, nu, length, collarLength;
double* poissonRatio;

public:
	Tube(double bendingStiffness, double PoissonsRatio, std::vector<Section> sections = std::vector<Section>());
	Tube(double bendingStiffness, double* PoissonsRatio, std::vector<Section> sections = std::vector<Section>());
	std::vector<Section>& GetSections ();
	double GetBendingStiffness() const {return this->kxy;};
	//double GetPoissonsRatio() const {return this->nu;};

	double GetPoissonsRatio() const 
	{
		if (this->poissonRatio)
			return *this->poissonRatio;

		return this->nu;
	}
	double GetTubeLength() const {return this->length;};
	double GetCollarLength() const {return this->collarLength;};


private:
    void UpdateLength ();
	void AddSection (Section section);


};
