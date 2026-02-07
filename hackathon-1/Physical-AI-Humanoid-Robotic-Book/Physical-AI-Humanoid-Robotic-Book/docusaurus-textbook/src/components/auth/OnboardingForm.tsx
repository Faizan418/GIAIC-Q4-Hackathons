import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { UserProfileCreate } from '../../types/chat';
import styles from './auth.module.css';

const OnboardingForm: React.FC = () => {
  const { state, updateProfile } = useAuth();

  const [formData, setFormData] = useState<UserProfileCreate>({
    software_level: 'BEGINNER',
    known_languages: [],
    hardware_experience: 'NONE',
    learning_track: 'FULL_ROBOTICS',
    boards_used: [],
  });

  const [step, setStep] = useState(1); // 3 steps for onboarding

  const handleLanguageChange = (language: string) => {
    setFormData(prev => {
      const languages = [...prev.known_languages];
      if (languages.includes(language)) {
        return {
          ...prev,
          known_languages: languages.filter(lang => lang !== language)
        };
      } else {
        return {
          ...prev,
          known_languages: [...languages, language]
        };
      }
    });
  };

  const handleBoardChange = (board: string) => {
    setFormData(prev => {
      const boards = [...prev.boards_used || []];
      if (boards.includes(board)) {
        return {
          ...prev,
          boards_used: boards.filter(b => b !== board)
        };
      } else {
        return {
          ...prev,
          boards_used: [...boards, board]
        };
      }
    });
  };

  const handleInputChange = (e: React.ChangeEvent<HTMLSelectElement | HTMLInputElement>) => {
    const { name, value, type } = e.target;
    if (type === 'select-one') {
      setFormData(prev => ({
        ...prev,
        [name]: value
      }));
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    try {
      // In a real implementation, we would call the API to save the profile
      // For now, we'll just update the local state
      console.log('Submitting profile:', formData);
      // updateProfile would be called with the created profile
    } catch (error) {
      console.error('Error submitting profile:', error);
    }
  };

  const languageOptions = ['Python', 'JavaScript', 'C++', 'Other'];
  const boardOptions = ['Arduino', 'ESP32', 'Raspberry Pi'];

  return (
    <div className={styles.authContainer}>
      <div className={styles.authForm}>
        <h2>Complete Your Profile</h2>
        <p>Tell us about your background to personalize your experience</p>

        <form onSubmit={handleSubmit}>
          {/* Step 1: Software Experience */}
          {step === 1 && (
            <div>
              <div className={styles.inputGroup}>
                <label htmlFor="software_level">Software Experience Level</label>
                <select
                  id="software_level"
                  name="software_level"
                  value={formData.software_level}
                  onChange={handleInputChange}
                  className={styles.select}
                >
                  <option value="BEGINNER">Beginner</option>
                  <option value="INTERMEDIATE">Intermediate</option>
                  <option value="ADVANCED">Advanced</option>
                </select>
              </div>

              <div className={styles.inputGroup}>
                <label>Programming Languages You Know</label>
                {languageOptions.map(lang => (
                  <label key={lang} className={styles.checkboxLabel}>
                    <input
                      type="checkbox"
                      checked={formData.known_languages.includes(lang)}
                      onChange={() => handleLanguageChange(lang)}
                    />
                    {lang}
                  </label>
                ))}
              </div>
            </div>
          )}

          {/* Step 2: Hardware Experience */}
          {step === 2 && (
            <div>
              <div className={styles.inputGroup}>
                <label htmlFor="hardware_experience">Hardware Experience Level</label>
                <select
                  id="hardware_experience"
                  name="hardware_experience"
                  value={formData.hardware_experience}
                  onChange={handleInputChange}
                  className={styles.select}
                >
                  <option value="NONE">No Experience</option>
                  <option value="BASIC">Basic</option>
                  <option value="INTERMEDIATE">Intermediate</option>
                  <option value="ADVANCED">Advanced</option>
                </select>
              </div>

              <div className={styles.inputGroup}>
                <label>Hardware Boards You've Used</label>
                {boardOptions.map(board => (
                  <label key={board} className={styles.checkboxLabel}>
                    <input
                      type="checkbox"
                      checked={(formData.boards_used || []).includes(board)}
                      onChange={() => handleBoardChange(board)}
                    />
                    {board}
                  </label>
                ))}
              </div>
            </div>
          )}

          {/* Step 3: Learning Track */}
          {step === 3 && (
            <div>
              <div className={styles.inputGroup}>
                <label htmlFor="learning_track">Learning Track</label>
                <select
                  id="learning_track"
                  name="learning_track"
                  value={formData.learning_track}
                  onChange={handleInputChange}
                  className={styles.select}
                >
                  <option value="SOFTWARE_ONLY">Software Only</option>
                  <option value="HARDWARE_ONLY">Hardware Only</option>
                  <option value="FULL_ROBOTICS">Full Robotics</option>
                </select>
              </div>
            </div>
          )}

          {/* Navigation buttons */}
          <div className={styles.buttonGroup}>
            {step > 1 && (
              <button
                type="button"
                onClick={() => setStep(step - 1)}
                className={styles.navButton}
              >
                Previous
              </button>
            )}
            {step < 3 ? (
              <button
                type="button"
                onClick={() => setStep(step + 1)}
                className={styles.navButton}
              >
                Next
              </button>
            ) : (
              <button
                type="submit"
                className={styles.submitButton}
              >
                Complete Profile
              </button>
            )}
          </div>
        </form>
      </div>
    </div>
  );
};

export default OnboardingForm;
