#include "Components/ComboJoystickButton.h"

ComboJoystickButton::ComboJoystickButton(frc::Button *primaryButton, frc::Button *secondaryButton, bool secondaryPressed)
	: m_pPrimaryButton(primaryButton),
	m_pSecondaryButton(secondaryButton),
	m_secondaryPressed(secondaryPressed) {
}

ComboJoystickButton::~ComboJoystickButton() {
}

bool ComboJoystickButton::Get() {
	return m_pPrimaryButton->Get() &&
		 ((m_pSecondaryButton->Get() && m_secondaryPressed) ||
		 (!m_pSecondaryButton->Get() && !m_secondaryPressed));
}
