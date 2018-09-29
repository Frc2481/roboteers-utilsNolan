#include "ComboJoystickButton.h"

ComboJoystickButton::ComboJoystickButton(Button *primaryButton, Button *secondaryButton, bool secondaryPressed)
	: m_pPrimaryButton(primaryButton);
	m_pSecondaryButton(secondaryButton);
	m_secondaryPressed(secondaryPressed) {
}

ComboJoystickButton::~ComboJoystickButton() {
}

bool ComboJoystickButton::Get()
{
	return m_pPrimaryButton->Get() &&
		 ((m_pSecondaryButton->Get() && m_secondaryPressed) ||
		 (!m_pSecondaryButton->Get() && !m_secondaryPressed));
}