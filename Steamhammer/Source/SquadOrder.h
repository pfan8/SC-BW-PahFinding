#pragma once

#include "Common.h"

namespace UAlbertaBot
{
	typedef std::vector<BWAPI::Position> SPosition;

namespace SquadOrderTypes
{
    enum {
		None,
		Idle,      // workers, overlords with no other job
		Attack,    // go attack
		Defend,    // defend a base (automatically disbanded when enemy is gone)
		Hold,      // hold ground, stand ready to defend until needed
		HoldWall,  // defend the wall
		Load,      // load into a transport (Drop squad)
		Drop,      // go drop on the enemy (Drop squad)
		Harass,    // harass the enemy
		KamikazeAttack,    // attacks the enemy with much higher aggression, ignoring air units
		SneakAttack, // detour to sneak enemy main base
	};
}

class SquadOrder
{
    size_t              _type;
    int                 _radius;
    BWAPI::Position     _dest_position;
	SPosition			_sneak_positions;
	int					_current_sneak_index;
    std::string         _status;

public:

	SquadOrder() 
		: _type(SquadOrderTypes::None)
        , _radius(0)
	{
	}

	SquadOrder(int type, BWAPI::Position dest_position, int radius, std::string status = "Default")
		: _type(type)
		, _dest_position(dest_position)
		, _radius(radius) 
		, _status(status)
	{
		if (type == SquadOrderTypes::SneakAttack)
		{
			_current_sneak_index = 0;
		}
	}

	void setSneakPostions(SPosition &sneak_positions)
	{
		_sneak_positions = sneak_positions;
	}

	const std::string & getStatus() const 
	{
		return _status;
	}

    const BWAPI::Position & getPosition() const
    {
        return _dest_position;
    }

    const int & getRadius() const
    {
        return _radius;
    }

    const size_t & getType() const
    {
        return _type;
    }

	const char getCharCode() const
	{
		switch (_type)
		{
			case SquadOrderTypes::None:    return '-';
			case SquadOrderTypes::Idle:    return 'I';
			case SquadOrderTypes::Attack:  return 'a';
			case SquadOrderTypes::Defend:  return 'd';
			case SquadOrderTypes::Hold:    return 'H';
			case SquadOrderTypes::HoldWall:return 'W';
			case SquadOrderTypes::Load:    return 'L';
			case SquadOrderTypes::Drop:    return 'D';
			case SquadOrderTypes::Harass:  return 'S';
			case SquadOrderTypes::KamikazeAttack:  return 'K';
		}
		return '?';
	}

	// These orders are considered combat orders and are linked to combat-related micro.
	bool isCombatOrder() const
	{
		return
			_type == SquadOrderTypes::Attack ||
			_type == SquadOrderTypes::Defend ||
			_type == SquadOrderTypes::Hold ||
			_type == SquadOrderTypes::HoldWall ||
			_type == SquadOrderTypes::Harass ||
			_type == SquadOrderTypes::KamikazeAttack ||
			_type == SquadOrderTypes::Drop;
	}

	// These orders use the regrouping mechanism to retreat when facing superior enemies.
	// Combat orders not in this group fight on against any odds.
	bool isRegroupableOrder() const
	{
		return
			_type == SquadOrderTypes::Attack;
	}

	BWAPI::Position getSneakPosition()
	{
		return _sneak_positions[_current_sneak_index];
	}

	void runNextSneakPos()
	{
		if (_sneak_positions.size() == 0) return;
		_current_sneak_index = _current_sneak_index >= _sneak_positions.size()-1 ? _sneak_positions.size()-1 : ++_current_sneak_index;
		//_current_sneak_index++;
	}

};
}