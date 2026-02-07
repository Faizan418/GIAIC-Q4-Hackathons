import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { auth } from '../../services/auth/better-auth-client';

function SignupPage() {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    softwareBackground: '',
    hardwareExperience: '',
    learningTrack: 'SOFTWARE_ONLY',
    skillLevel: 'BEGINNER'
  });
  const [error, setError] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);

  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLTextAreaElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/auth/signup', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(formData),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.error || 'Signup failed');
      }

      // Wait a brief moment to ensure session is established
      await new Promise(resolve => setTimeout(resolve, 300));

      // Redirect to dashboard after successful signup
      window.location.href = '/dashboard';
    } catch (err: any) {
      setError(err.message);
      setLoading(false);
    }
  };

  return (
    <Layout title="Sign Up" description="Create your account">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <div className="card">
              <div className="card__header">
                <h2>Create Account</h2>
              </div>
              <div className="card__body">
                {error && (
                  <div className="alert alert--danger" role="alert">
                    {error}
                  </div>
                )}
                <form onSubmit={handleSubmit}>
                  <div className="form-group margin-bottom--md">
                    <label htmlFor="email">Email</label>
                    <input
                      type="email"
                      id="email"
                      name="email"
                      value={formData.email}
                      onChange={handleChange}
                      required
                      className="form-control"
                      disabled={loading}
                    />
                  </div>

                  <div className="form-group margin-bottom--md">
                    <label htmlFor="password">Password</label>
                    <input
                      type="password"
                      id="password"
                      name="password"
                      value={formData.password}
                      onChange={handleChange}
                      required
                      className="form-control"
                      disabled={loading}
                    />
                  </div>

                  <div className="form-group margin-bottom--md">
                    <label htmlFor="softwareBackground">Software Background</label>
                    <textarea
                      id="softwareBackground"
                      name="softwareBackground"
                      value={formData.softwareBackground}
                      onChange={handleChange}
                      placeholder="e.g., ROS2, Python, JavaScript, C++, etc."
                      className="form-control"
                      disabled={loading}
                    />
                  </div>

                  <div className="form-group margin-bottom--md">
                    <label htmlFor="hardwareExperience">Hardware/Robotics Experience</label>
                    <textarea
                      id="hardwareExperience"
                      name="hardwareExperience"
                      value={formData.hardwareExperience}
                      onChange={handleChange}
                      placeholder="e.g., Arduino, ESP32, Raspberry Pi, electronics experience, etc."
                      className="form-control"
                      disabled={loading}
                    />
                  </div>

                  <div className="form-group margin-bottom--md">
                    <label htmlFor="learningTrack">Learning Track</label>
                    <select
                      id="learningTrack"
                      name="learningTrack"
                      value={formData.learningTrack}
                      onChange={handleChange}
                      className="form-control"
                      disabled={loading}
                    >
                      <option value="SOFTWARE_ONLY">Software Only</option>
                      <option value="HARDWARE_ONLY">Hardware Only</option>
                      <option value="FULL_ROBOTICS">Full Robotics</option>
                    </select>
                  </div>

                  <div className="form-group margin-bottom--md">
                    <label htmlFor="skillLevel">Skill Level</label>
                    <select
                      id="skillLevel"
                      name="skillLevel"
                      value={formData.skillLevel}
                      onChange={handleChange}
                      className="form-control"
                      disabled={loading}
                    >
                      <option value="BEGINNER">Beginner</option>
                      <option value="INTERMEDIATE">Intermediate</option>
                      <option value="ADVANCED">Advanced</option>
                    </select>
                  </div>

                  <button
                    type="submit"
                    className="button button--primary button--block"
                    disabled={loading}
                  >
                    {loading ? 'Creating Account...' : 'Sign Up'}
                  </button>
                </form>
              </div>
              <div className="card__footer">
                <p>
                  Already have an account? <a href="/auth/signin">Sign in</a>
                </p>
              </div>
            </div>
          </div>
        </div>
      </div>
    </Layout>
  );
}

export default SignupPage;
