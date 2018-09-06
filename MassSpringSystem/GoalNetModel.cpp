#include "GoalNetModel.h"
#include <iostream>
#include "configFile.h"

const double g_cdK = 2500.0f;
const double g_cdD = 50.0f;

GoalNet::GoalNet()
:m_InitPos(Vector3d(0.0, 1.0, 0.0)),
m_NetWidth(2.0),
m_NetHeight(3.0),
m_NetLength(8.0),
m_NumAtWidth(10),
m_NumAtHeight(20),
m_NumAtLength(35),
m_Particles(),
m_Springs(),
m_ParticleIdMap(),
m_dSpringCoefStruct(g_cdK),
m_dSpringCoefShear(g_cdK),
m_dSpringCoefBending(g_cdK),
m_dDamperCoefStruct(g_cdD),
m_dDamperCoefShear(g_cdD),
m_dDamperCoefBending(g_cdD),
m_ColorStruct(Vector3d(0.8,0.8,0.8)),
m_ColorShear(Vector3d(0.0,0.0,0.0)),
m_ColorBending(Vector3d(0.0,0.0,0.0))
{
    Initialize();
}

GoalNet::GoalNet(const GoalNet &a_rcGoalNet)
:m_InitPos(a_rcGoalNet.m_InitPos),
m_NetWidth(a_rcGoalNet.m_NetWidth),
m_NetHeight(a_rcGoalNet.m_NetHeight),
m_NetLength(a_rcGoalNet.m_NetLength),
m_NumAtWidth(a_rcGoalNet.m_NumAtWidth),
m_NumAtHeight(a_rcGoalNet.m_NumAtHeight),
m_NumAtLength(a_rcGoalNet.m_NumAtLength),
m_Particles(a_rcGoalNet.m_Particles),
m_Springs(a_rcGoalNet.m_Springs),
m_ParticleIdMap(a_rcGoalNet.m_ParticleIdMap),
m_dSpringCoefStruct(a_rcGoalNet.m_dSpringCoefStruct),
m_dSpringCoefShear(a_rcGoalNet.m_dSpringCoefShear),
m_dSpringCoefBending(a_rcGoalNet.m_dSpringCoefBending),
m_dDamperCoefStruct(a_rcGoalNet.m_dDamperCoefStruct),
m_dDamperCoefShear(a_rcGoalNet.m_dDamperCoefShear),
m_dDamperCoefBending(a_rcGoalNet.m_dDamperCoefBending),
m_ColorStruct(a_rcGoalNet.m_ColorStruct),
m_ColorShear(a_rcGoalNet.m_ColorShear),
m_ColorBending(a_rcGoalNet.m_ColorBending)
{
    Initialize();
}

GoalNet::GoalNet(const std::string &a_rcsConfigFilename)
:m_ColorStruct(Vector3d(0.8, 0.8, 0.8)),
m_ColorShear(Vector3d(0.0, 0.0, 0.0)),
m_ColorBending(Vector3d(0.0, 0.0, 0.0))
{
    ConfigFile configFile;
    configFile.suppressWarnings(1);

    configFile.addOption("NetInitPos_x", &m_InitPos.x);
    configFile.addOption("NetInitPos_y", &m_InitPos.y);
    configFile.addOption("NetInitPos_z", &m_InitPos.z);
    configFile.addOption("NetWidth", &m_NetWidth);
    configFile.addOption("NetHeight", &m_NetHeight);
    configFile.addOption("NetLength", &m_NetLength);
    configFile.addOption("NumAtWidth", &m_NumAtWidth);
    configFile.addOption("NumAtHeight", &m_NumAtHeight);
    configFile.addOption("NumAtLength", &m_NumAtLength);
    configFile.addOption("SpringCoef", &m_dSpringCoefStruct);
    configFile.addOption("SpringCoef", &m_dSpringCoefShear);
    configFile.addOption("SpringCoef", &m_dSpringCoefBending);
    configFile.addOption("DamperCoef", &m_dDamperCoefStruct);
    configFile.addOption("DamperCoef", &m_dDamperCoefStruct);
    configFile.addOption("DamperCoef", &m_dDamperCoefStruct);

    int code = configFile.parseOptions((char *)a_rcsConfigFilename.c_str());
    if (code == 1)
    {
        std::cout << "Error in CMassSpringSystem constructor." << std::endl;
        system("pause");
        exit(0);
    }

    Initialize();
}

GoalNet::~GoalNet()
{
}

CParticle& GoalNet::GetParticle(int particleIdx)
{
    return m_Particles[particleIdx];
}

CSpring& GoalNet::GetSpring(int springIdx)
{
    return m_Springs[springIdx];
}

int GoalNet::ParticleNum() const
{
    return m_Particles.size();
}

int GoalNet::SpringNum() const
{
    return m_Springs.size();
}

int GoalNet::GetWidthNum() const
{
    return m_NumAtWidth;
}

int GoalNet::GetHeightNum() const
{
    return m_NumAtHeight;
}

int GoalNet::GetLengthNum() const
{
    return m_NumAtLength;
}

int GoalNet::GetParticleID(
    int xId,
    int yId,
    int zId
    )
{
    int numAtBack = m_NumAtHeight * m_NumAtLength;
    int cuboidIdx = xId*numAtBack + yId*m_NumAtLength + zId;
    if (m_ParticleIdMap.find(cuboidIdx) != m_ParticleIdMap.end())
    {
        return m_ParticleIdMap.find(cuboidIdx)->second;
    }
    else
    {
        std::cout << "no such particle exists!" << std::endl;
        return -1;
    }
}

Vector3d GoalNet::GetInitPos() const
{
    return m_InitPos;
}

void GoalNet::SetSpringCoef(
    const double a_cdSpringCoef,
    const CSpring::enType_t a_cSpringType
    )
{
    if (a_cSpringType == CSpring::Type_nStruct)
    {
        m_dSpringCoefStruct = a_cdSpringCoef;
        UpdateSpringCoef(a_cdSpringCoef, CSpring::Type_nStruct);
    }
    else if (a_cSpringType == CSpring::Type_nShear)
    {
        m_dSpringCoefShear = a_cdSpringCoef;
        UpdateSpringCoef(a_cdSpringCoef, CSpring::Type_nShear);
    }
    else if (a_cSpringType == CSpring::Type_nBending)
    {
        m_dSpringCoefBending = a_cdSpringCoef;
        UpdateSpringCoef(a_cdSpringCoef, CSpring::Type_nBending);
    }
}

void GoalNet::SetDamperCoef(
    const double a_cdDamperCoef,
    const CSpring::enType_t a_cSpringType
    )
{
    if (a_cSpringType == CSpring::Type_nStruct)
    {
        m_dDamperCoefStruct = a_cdDamperCoef;
        UpdateDamperCoef(a_cdDamperCoef, CSpring::Type_nStruct);
    }
    else if (a_cSpringType == CSpring::Type_nShear)
    {
        m_dDamperCoefShear = a_cdDamperCoef;
        UpdateDamperCoef(a_cdDamperCoef, CSpring::Type_nShear);
    }
    else if (a_cSpringType == CSpring::Type_nBending)
    {
        m_dDamperCoefBending = a_cdDamperCoef;
        UpdateDamperCoef(a_cdDamperCoef, CSpring::Type_nBending);
    }
}

void GoalNet::Reset()
{
    int numAtBack = m_NumAtHeight * m_NumAtLength;
    for (int i = 0; i < m_NumAtWidth; ++i)
    {
        for (int j = 0; j < m_NumAtHeight; ++j)
        {
            for (int k = 0; k < m_NumAtLength; ++k)
            {
                if (isAtFace(i, j, k))   // at the four faces in the goal net
                {
                    double offset_x = (double)((i - m_NumAtWidth / 2) * m_NetWidth / (m_NumAtWidth - 1));
                    double offset_y = (double)((j - m_NumAtHeight / 2) * m_NetHeight / (m_NumAtHeight - 1));
                    double offset_z = (double)((k - m_NumAtLength / 2) * m_NetLength / (m_NumAtLength - 1));
                    
                    int pIdx = GetParticleID(i, j, k);
                    m_Particles[pIdx].SetPosition(
                        Vector3d(
                            m_InitPos.x + offset_x,
                            m_InitPos.y + offset_y,
                            m_InitPos.z + offset_z
                            )
                        );
                    m_Particles[pIdx].SetForce(Vector3d::ZERO);
                    m_Particles[pIdx].SetVelocity(Vector3d::ZERO);
                }
            }
        }
    }
}

void GoalNet::AddForceField(const Vector3d &a_kForce)
{
    for (unsigned int uiI = 0; uiI < m_Particles.size(); uiI++)
    {
        //m_Particles[uiI].SetAcceleration(a_kForce);
        m_Particles[uiI].AddForce(a_kForce * m_Particles[uiI].GetMass());
    }
}

void GoalNet::ComputeInternalForce()
{
    //TODO 2
	for (int i = 0; i < m_Springs.size(); i++)
	{
		CParticle par1 = GetParticle(m_Springs[i].GetSpringStartID());
		CParticle par2 = GetParticle(m_Springs[i].GetSpringEndID());

		Vector3d damperForceToPar1 = ComputeDamperForce( par1.GetPosition(), par2.GetPosition(), par1.GetVelocity(), par2.GetVelocity(), m_Springs[i].GetDamperCoef());
		Vector3d springForceToPar1 = ComputeSpringForce(par1.GetPosition(), par2.GetPosition(), m_Springs[i].GetSpringCoef(), m_Springs[i].GetSpringRestLength());

		Vector3d damperForceToPar2 = ComputeDamperForce(par2.GetPosition(), par1.GetPosition(), par2.GetVelocity(), par1.GetVelocity(), m_Springs[i].GetDamperCoef());
		Vector3d springForceToPar2 = ComputeSpringForce(par2.GetPosition(), par1.GetPosition(), m_Springs[i].GetSpringCoef(), m_Springs[i].GetSpringRestLength());

		GetParticle(m_Springs[i].GetSpringStartID()).AddForce(damperForceToPar1 + springForceToPar1);
		GetParticle(m_Springs[i].GetSpringEndID()).AddForce(damperForceToPar2 + springForceToPar2);
	}
}

/*
 * private function
 */
void GoalNet::Initialize()
{
    InitializeParticle();
    InitializeSpring();
}

void GoalNet::InitializeParticle()
{
    int numAtBack = m_NumAtHeight * m_NumAtLength;
    for (int i = 0; i < m_NumAtWidth; ++i)
    {
        for (int j = 0; j < m_NumAtHeight; ++j)
        {
            for (int k = 0; k < m_NumAtLength; ++k)
            {
                if (isAtFace(i, j, k))   // at the four faces in the goal net
                {
                    CParticle Particle;
                    double offset_x = (double)( (i - m_NumAtWidth/2) * m_NetWidth / (m_NumAtWidth-1) );
                    double offset_y = (double)( (j - m_NumAtHeight/2) * m_NetHeight / (m_NumAtHeight-1) );
                    double offset_z = (double)( (k - m_NumAtLength/2) * m_NetLength / (m_NumAtLength-1) );
                    Particle.SetPosition(
                        Vector3d(
                            m_InitPos.x + offset_x,
                            m_InitPos.y + offset_y,
                            m_InitPos.z + offset_z
                            )
                        );
                    Particle.SetMass(0.2);
                    if (isAtEdge(i, j, k))
                    {
                        Particle.SetMovable(false);
                    }
                    m_Particles.push_back(Particle);
                    int cuboidIdx = i*numAtBack + j*m_NumAtLength + k;
                    m_ParticleIdMap.insert( 
                        pair<int, int>(cuboidIdx, m_Particles.size()-1) 
                        );
                }
            }
        }
    }
}

void GoalNet::InitializeSpring()
{
    int startParticleId = 0;
    int endParticleId = 0;
	// TODO 1
	//(Width, Height, Length)

		for (int j = 0; j < m_NumAtHeight; j++){
			for (int i = 0; i < m_NumAtWidth - 1; i++){
				
				//Face 1
				startParticleId = GetParticleID(i, j, 0);
				endParticleId = GetParticleID(i + 1, j, 0);
				addStructSpring(startParticleId, endParticleId);
				//Face 2
				startParticleId = GetParticleID(i, j, m_NumAtLength - 1);
				endParticleId = GetParticleID(i + 1, j, m_NumAtLength - 1);
				addStructSpring(startParticleId, endParticleId);
			}
		}

		for (int j = 0; j < m_NumAtHeight; j++){
			for (int i = 0; i < m_NumAtLength - 1; i++){
				//Face 1
				startParticleId = GetParticleID(0, j, i);
				endParticleId = GetParticleID(0, j, i + 1);
				addStructSpring(startParticleId, endParticleId);
				//Face 2
				startParticleId = GetParticleID(m_NumAtWidth - 1, j, i);
				endParticleId = GetParticleID(m_NumAtWidth - 1, j, i + 1);
				addStructSpring(startParticleId, endParticleId);
			}
		}

		for (int j = 0; j < m_NumAtLength; j++){
			for (int i = 0; i < m_NumAtHeight - 1; i++){
				//Face 1
				startParticleId = GetParticleID(0, i, j);
				endParticleId = GetParticleID(0, i + 1, j);
				addStructSpring(startParticleId, endParticleId);
				//Face 2
				startParticleId = GetParticleID(m_NumAtWidth - 1, i, j);
				endParticleId = GetParticleID(m_NumAtWidth - 1, i + 1, j);
				addStructSpring(startParticleId, endParticleId);
			}
		}

		for (int j = 0; j < m_NumAtWidth; j++){
			for (int i = 0; i < m_NumAtHeight - 1; i++){
				//Face 1
				startParticleId = GetParticleID(j, i, 0);
				endParticleId = GetParticleID(j, i + 1, 0);
				addStructSpring(startParticleId, endParticleId);
				//Face 2
				startParticleId = GetParticleID(j, i, m_NumAtLength - 1);
				endParticleId = GetParticleID(j, i + 1, m_NumAtLength - 1);
				addStructSpring(startParticleId, endParticleId);
			}
		}

		for (int j = 0; j < m_NumAtWidth; j++){
			for (int i = 0; i < m_NumAtLength - 1; i++){
				//Bottom Vertical
				startParticleId = GetParticleID(j, 0, i);
				endParticleId = GetParticleID(j, 0, i + 1);
				addStructSpring(startParticleId, endParticleId);
			}
		}
		for (int j = 0; j < m_NumAtLength; j++){
			for (int i = 0; i < m_NumAtWidth - 1; i++){
				//Bottom Horizontal
				startParticleId = GetParticleID(i, 0, j);
				endParticleId = GetParticleID(i + 1, 0, j);
				addStructSpring(startParticleId, endParticleId);
			}
		}

    std::cout << "total num of springs:" << std::endl;
    std::cout << this->SpringNum() << std::endl;
}

void GoalNet::addStructSpring(
    int startParticleId,
    int endParticleId
    )
{
    Vector3d startPos = m_Particles[startParticleId].GetPosition();
    Vector3d endPos = m_Particles[endParticleId].GetPosition();
    Vector3d offsetVec = startPos - endPos;
    CSpring SpringStruct(
        startParticleId,
        endParticleId,
        offsetVec.Length(),
        m_dSpringCoefStruct,
        m_dDamperCoefStruct,
        m_ColorStruct,
        CSpring::Type_nStruct);
    m_Springs.push_back(SpringStruct);
}

Vector3d GoalNet::ComputeSpringForce(
    const Vector3d &a_crPos1,
    const Vector3d &a_crPos2,
    const double a_cdSpringCoef,
    const double a_cdRestLength
    )
{
    //TODO 3 Spring Force
	Vector3d force;
	Vector3d diff = (a_crPos1 - a_crPos2);
	force = - a_cdSpringCoef * (diff.Length() - a_cdRestLength) * diff.NormalizedCopy();
	return force;
}

Vector3d GoalNet::ComputeDamperForce(
    const Vector3d &a_crPos1,
    const Vector3d &a_crPos2,
    const Vector3d &a_crVel1,
    const Vector3d &a_crVel2,
    const double a_cdDamperCoef
    )
{
    //TODO 4 Damper Force
	Vector3d force;
	Vector3d lVector = (a_crPos1 - a_crPos2).NormalizedCopy();
	force = - a_cdDamperCoef * ((a_crVel1 - a_crVel2).DotProduct(a_crPos1 - a_crPos2) / (a_crPos1 - a_crPos2).Length()) *lVector;
	return force;
}

void GoalNet::UpdateSpringCoef(
    const double a_cdSpringCoef,
    const CSpring::enType_t a_cSpringType
    )
{
    for (unsigned int uiI = 0; uiI < m_Springs.size(); uiI++)
    {
        if (m_Springs[uiI].GetSpringType() == a_cSpringType)
        {
            m_Springs[uiI].SetSpringCoef(m_dSpringCoefStruct);
        }
        else if (m_Springs[uiI].GetSpringType() == a_cSpringType)
        {
            m_Springs[uiI].SetSpringCoef(m_dSpringCoefShear);
        }
        else if (m_Springs[uiI].GetSpringType() == a_cSpringType)
        {
            m_Springs[uiI].SetSpringCoef(m_dSpringCoefBending);
        }
    }

}

void GoalNet::UpdateDamperCoef(
    const double a_cdDamperCoef,
    const CSpring::enType_t a_cSpringType
    )
{
    for (unsigned int uiI = 0; uiI<m_Springs.size(); uiI++)
    {
        if (m_Springs[uiI].GetSpringType() == a_cSpringType)
        {
            m_Springs[uiI].SetDamperCoef(m_dDamperCoefStruct);
        }
        else if (m_Springs[uiI].GetSpringType() == a_cSpringType)
        {
            m_Springs[uiI].SetDamperCoef(m_dDamperCoefShear);
        }
        else if (m_Springs[uiI].GetSpringType() == a_cSpringType)
        {
            m_Springs[uiI].SetDamperCoef(m_dDamperCoefBending);
        }
    }
}

bool GoalNet::isAtFace(
    const int xId,
    const int yId,
    const int zId
    )
{
    // Is the position at the five faces in the goal net
    if (xId == 0 || xId == m_NumAtWidth - 1 || yId == 0 || zId == 0 || zId == m_NumAtLength - 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool GoalNet::isAtEdge(
    const int xId,
    const int yId,
    const int zId
    )
{
    // at top face
    if (yId == m_NumAtHeight - 1)   
    {
        if (xId == 0 || xId == m_NumAtWidth - 1 || zId == 0 || zId == m_NumAtLength - 1)
        {
            return true;
        }
    }
    // at bottom face
	/*
    if (yId == 0)   
    {
        if (xId == 0 || zId == 0 || zId == m_NumAtLength - 1)
        {
            return true;
        }
    }
	*/
    // at side face
    if (zId == 0)   
    {
        if (xId == 0 || xId == m_NumAtWidth - 1)
        {
            return true;
        }
    }
    if (zId == m_NumAtLength - 1)
    {
        if (xId == 0 || xId == m_NumAtWidth - 1)
        {
            return true;
        }
    }
    return false;
}