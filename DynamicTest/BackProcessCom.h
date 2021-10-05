#ifndef BACKPROCESSCOM_H
#define BACKPROCESSCOM_H

#include <string>

class BackProcessCom
{
public:
	BackProcessCom() {}

	virtual ~BackProcessCom() {}

	virtual int initial_parameter(const std::string & config_filename) = 0;

	virtual int registration() = 0;

	virtual int searching() = 0;

	virtual int measurement() = 0;

};

#endif // !BACKPROCESSCOM_H
