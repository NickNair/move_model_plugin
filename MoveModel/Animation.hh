#include <gz/math/Spline.hh>
#include <gz/math/RotationSpline.hh>

#include <gz/common/Export.hh>

#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>
class KeyFrame
    {
      /// \brief Constructor
      /// \param[in] _time Time of the keyframe in seconds
      public: explicit KeyFrame(const double _time);

      /// \brief Destructor
      public: virtual ~KeyFrame();

      /// \brief Get the time of the keyframe
      /// \return the time
      public: double Time() const;

      /// \brief time of key frame
      protected: double time;
    };

    /// \brief A keyframe for a PoseAnimation
    class PoseKeyFrame : public KeyFrame
    {
      /// \brief Constructor
      /// \param[in] _time of the keyframe
      public: explicit PoseKeyFrame(const double _time);

      /// \brief Destructor
      public: virtual ~PoseKeyFrame();

      /// \brief Set the translation for the keyframe
      /// \param[in] _trans Translation amount
      public: void Translation(const gz::math::Vector3d &_trans);

      /// \brief Get the translation of the keyframe
      /// \return The translation amount
      public: const gz::math::Vector3d &Translation() const;

      /// \brief Set the rotation for the keyframe
      /// \param[in] _rot Rotation amount
      public: void Rotation(const gz::math::Quaterniond &_rot);

      /// \brief Get the rotation of the keyframe
      /// \return The rotation amount
      public: const gz::math::Quaterniond &Rotation() const;

      /// \brief the translation vector
      protected: gz::math::Vector3d translate;

      /// \brief the rotation quaternion
      protected: gz::math::Quaterniond rotate;

    };

    /// \brief A keyframe for a NumericAnimation
    class NumericKeyFrame : public KeyFrame
    {
      /// \brief Constructor
      /// \param[in] _time Time of the keyframe
      public: explicit NumericKeyFrame(const double _time);

      /// \brief Destructor
      public: virtual ~NumericKeyFrame();

      /// \brief Set the value of the keyframe
      /// \param[in] _value The new value
      public: void Value(const double &_value);

      /// \brief Get the value of the keyframe
      /// \return the value of the keyframe
      public: const double &Value() const;

      /// \brief numeric value
      protected: double value;
    };
    /// \class Animation Animation.hh ignition/common/Animation.hh
    /// \brief Manages an animation, which is a collection of keyframes and
    /// the ability to interpolate between the keyframes
    class  Animation
    {
      /// \brief Constructor
      /// \param[in] _name Name of the animation, should be unique
      /// \param[in] _length Duration of the animation in seconds
      /// \param[in] _loop Set to true if the animation should repeat
      public: Animation(const std::string &_name,
                  const double _length, const bool _loop);

      /// \brief Destructor
      public: virtual ~Animation();

      /// \brief Return the duration of the animation
      /// \return Duration of the animation in seconds
      public: double Length() const;

      /// \brief Set the duration of the animation
      /// \param[in] _len The length of the animation in seconds
      public: void Length(const double _len);

      /// \brief Set the current time position of the animation
      /// \param[in] _time The time position in seconds
      public: void Time(const double _time);

      /// \brief Add time to the animation
      /// \param[in] _time The amount of time to add in seconds
      public: void AddTime(const double _time);

      /// \brief Return the current time position
      /// \return The time position in seconds
      public: double Time() const;

      /// \brief Return the number of key frames in the animation
      /// \return The number of keyframes
      public: unsigned int KeyFrameCount() const;

      /// \brief Get a key frame using an index value
      /// \param[in] _index The index of the key frame
      /// \return A pointer the keyframe, NULL if the _index is invalid
      public: KeyFrame *GetKeyFrame(const unsigned int _index) const;

      /// \brief Get the two key frames that bound a time value
      /// \param[in] _time The time in seconds
      /// \param[out] _kf1 Lower bound keyframe that is returned
      /// \param[out] _kf2 Upper bound keyframe that is returned
      /// \param[out] _firstKeyIndex Index of the lower bound key frame
      /// \return The time between the two keyframe
      protected: double KeyFramesAtTime(
                     double _time, KeyFrame **_kf1,
                     KeyFrame **_kf2,
                     unsigned int &_firstKeyIndex) const;

      /// \brief animation name
      protected: std::string name;

      /// \brief animation duration
      protected: double length;

      /// \brief current time position
      protected: double timePos;

      /// \brief determines if the interpolation splines need building
      protected: mutable bool build;

      /// \brief true if animation repeats
      protected: bool loop;

      /// \brief array of keyframe type alias
      protected: typedef std::vector<KeyFrame*> KeyFrame_V;

      /// \brief array of key frames
      protected: KeyFrame_V keyFrames;

    };

    /// \brief A pose animation.
    class  PoseAnimation : public Animation
    {
      /// \brief Constructor
      /// \param[in] _name String name of the animation. This should be unique.
      /// \param[in] _length Length of the animation in seconds
      /// \param[in] _loop True == loop the animation
      public: PoseAnimation(const std::string &_name,
                            const double _length, const bool _loop);

      /// \brief Destructor
      public: virtual ~PoseAnimation();

      /// \brief Create a pose keyframe at the given time
      /// \param[in] _time Time at which to create the keyframe
      /// \return Pointer to the new keyframe
      public: PoseKeyFrame *CreateKeyFrame(const double _time);

      /// \brief Get a keyframe using the animation's current time.
      /// \param[out] _kf PoseKeyFrame reference to hold the interpolated result
      public: void InterpolatedKeyFrame(PoseKeyFrame &_kf) const;

      /// \brief Get a keyframe using a passed in time.
      /// \param[in] _time Time in seconds
      /// \param[out] _kf PoseKeyFrame reference to hold the interpolated result
      protected: void InterpolatedKeyFrame(const double _time,
                                              PoseKeyFrame &_kf) const;

      /// \brief Update the pose splines
      protected: void BuildInterpolationSplines() const;

      /// \brief smooth interpolation for position
      private: mutable gz::math::Spline *positionSpline;

      /// \brief smooth interpolation for rotation
      private: mutable gz::math::RotationSpline *rotationSpline;
    };

    /// \brief A numeric animation.
    class  NumericAnimation : public Animation
    {
      /// \brief Constructor
      /// \param[in] _name String name of the animation. This should be unique.
      /// \param[in] _length Length of the animation in seconds
      /// \param[in] _loop True == loop the animation
      public: NumericAnimation(const std::string &_name,
                               const double _length, const bool _loop);

      /// \brief Destructor
      public: virtual ~NumericAnimation();

      /// \brief Create a numeric keyframe at the given time
      /// \param[in] _time Time at which to create the keyframe
      /// \return Pointer to the new keyframe
      public: NumericKeyFrame *CreateKeyFrame(const double _time);

      /// \brief Get a keyframe using the animation's current time.
      /// \param[out] _kf NumericKeyFrame reference to hold the
      /// interpolated result
      public: void InterpolatedKeyFrame(NumericKeyFrame &_kf) const;
    };

struct KeyFrameTimeLess
  {
    bool operator() (const KeyFrame *_kf,
        const KeyFrame *_kf2) const
    {
      return _kf->Time() < _kf2->Time();
    }
  };


KeyFrame::KeyFrame(const double _time)
: time(_time)
{
}

/////////////////////////////////////////////////
KeyFrame::~KeyFrame()
{
}

/////////////////////////////////////////////////
double KeyFrame::Time() const
{
  return this->time;
}


/////////////////////////////////////////////////
PoseKeyFrame::PoseKeyFrame(const double _time)
: KeyFrame(_time)
{
}

/////////////////////////////////////////////////
PoseKeyFrame::~PoseKeyFrame()
{
}

/////////////////////////////////////////////////
void PoseKeyFrame::Translation(const gz::math::Vector3d &_trans)
{
  this->translate = _trans;
}

/////////////////////////////////////////////////
const gz::math::Vector3d &PoseKeyFrame::Translation() const
{
  return this->translate;
}

/////////////////////////////////////////////////
void PoseKeyFrame::Rotation(const gz::math::Quaterniond &_rot)
{
  this->rotate = _rot;
}

/////////////////////////////////////////////////
const gz::math::Quaterniond &PoseKeyFrame::Rotation() const
{
  return this->rotate;
}

/////////////////////////////////////////////////
NumericKeyFrame::NumericKeyFrame(const double _time)
: KeyFrame(_time)
{
}

/////////////////////////////////////////////////
NumericKeyFrame::~NumericKeyFrame()
{
}

/////////////////////////////////////////////////
void NumericKeyFrame::Value(const double &_value)
{
  this->value = _value;
}

/////////////////////////////////////////////////
const double &NumericKeyFrame::Value() const
{
  return this->value;
}

Animation::Animation(const std::string &_name, const double _length,
    const bool _loop)
: name(_name), length(_length), loop(_loop)
{
  this->timePos = 0;
  this->build = false;
}

/////////////////////////////////////////////////
Animation::~Animation()
{
}

/////////////////////////////////////////////////
double Animation::Length() const
{
  return this->length;
}

/////////////////////////////////////////////////
void Animation::Length(const double _len)
{
  this->length = _len;
}

/////////////////////////////////////////////////
void Animation::Time(const double _time)
{
  if (!gz::math::equal(_time, this->timePos))
  {
    this->timePos = _time;
    if (this->loop)
    {
      this->timePos = fmod(this->timePos, this->length);
      if (this->timePos < 0)
        this->timePos += this->length;
    }
    else
    {
      if (this->timePos < 0)
        this->timePos = 0;
      else if (this->timePos > this->length)
        this->timePos = this->length;
    }
  }
}

/////////////////////////////////////////////////
void Animation::AddTime(const double _time)
{
  this->Time(this->timePos + _time);
}

/////////////////////////////////////////////////
double Animation::Time() const
{
  return this->timePos;
}

/////////////////////////////////////////////////
unsigned int Animation::KeyFrameCount() const
{
  return this->keyFrames.size();
}

/////////////////////////////////////////////////
KeyFrame *Animation::GetKeyFrame(const unsigned int _index) const
{
  KeyFrame *result = NULL;

  if (_index < this->keyFrames.size())
    result = this->keyFrames[_index];
  else
  {
    gzerr << "Key frame index[" << _index
          << "] is larger than key frame array size["
          << this->keyFrames.size() << "]\n";
  }

  return result;
}

/////////////////////////////////////////////////
double Animation::KeyFramesAtTime(double _time, KeyFrame **_kf1,
    KeyFrame **_kf2, unsigned int &_firstKeyIndex) const
{
  // Parametric time
  // t1 = time of previous keyframe
  // t2 = time of next keyframe
  double t1, t2;

  // Find first key frame after or on current time
  while (_time > this->length && this->length > 0.0)
    _time -= this->length;

  KeyFrame_V::const_iterator iter;
  KeyFrame timeKey(_time);
  iter = std::lower_bound(this->keyFrames.begin(), this->keyFrames.end(),
      &timeKey, KeyFrameTimeLess());

  if (iter == this->keyFrames.end())
  {
    // There is no keyframe after this time, wrap back to first
    *_kf2 = this->keyFrames.front();
    t2 = this->length + (*_kf2)->Time();

    // Use the last keyframe as the previous keyframe
    --iter;
  }
  else
  {
    *_kf2 = *iter;
    t2 = (*_kf2)->Time();

    // Find last keyframe before or on current time
    if (iter != this->keyFrames.begin() && _time < (*iter)->Time())
      --iter;
  }

  _firstKeyIndex = std::distance(this->keyFrames.begin(), iter);

  *_kf1 = *iter;
  t1 = (*_kf1)->Time();

  if (gz::math::equal(t1, t2))
    return 0.0;
  else
    return (_time - t1) / (t2 - t1);
}

/////////////////////////////////////////////////
PoseAnimation::PoseAnimation(const std::string &_name,
    const double _length, const bool _loop)
: Animation(_name, _length, _loop)
{
  this->positionSpline = NULL;
  this->rotationSpline = NULL;
}

/////////////////////////////////////////////////
PoseAnimation::~PoseAnimation()
{
  delete this->positionSpline;
  delete this->rotationSpline;
}

/////////////////////////////////////////////////
PoseKeyFrame *PoseAnimation::CreateKeyFrame(const double _time)
{
  PoseKeyFrame *frame = new PoseKeyFrame(_time);
  std::vector<KeyFrame*>::iterator iter =
    std::upper_bound(this->keyFrames.begin(), this->keyFrames.end(),
        reinterpret_cast<KeyFrame*>(frame), KeyFrameTimeLess());

  this->keyFrames.insert(iter, frame);
  this->build = true;

  return frame;
}

/////////////////////////////////////////////////
void PoseAnimation::BuildInterpolationSplines() const
{
  if (!this->positionSpline)
    this->positionSpline = new gz::math::Spline();

  if (!this->rotationSpline)
    this->rotationSpline = new gz::math::RotationSpline();

  this->positionSpline->AutoCalculate(false);
  this->rotationSpline->AutoCalculate(false);

  this->positionSpline->Clear();
  this->rotationSpline->Clear();

  for (KeyFrame_V::const_iterator iter = this->keyFrames.begin();
      iter != this->keyFrames.end(); ++iter)
  {
    PoseKeyFrame *pkey = reinterpret_cast<PoseKeyFrame*>(*iter);
    this->positionSpline->AddPoint(pkey->Translation());
    this->rotationSpline->AddPoint(pkey->Rotation());
  }

  this->positionSpline->RecalcTangents();
  this->rotationSpline->RecalcTangents();
  this->build = false;
}

/////////////////////////////////////////////////
void PoseAnimation::InterpolatedKeyFrame(PoseKeyFrame &_kf) const
{
  this->InterpolatedKeyFrame(this->timePos, _kf);
}

/////////////////////////////////////////////////
void PoseAnimation::InterpolatedKeyFrame(const double _time,
                                         PoseKeyFrame &_kf) const
{
  KeyFrame *kBase1, *kBase2;
  PoseKeyFrame *k1;
  unsigned int firstKeyIndex;

  if (this->build)
    this->BuildInterpolationSplines();

  double t = this->KeyFramesAtTime(_time, &kBase1, &kBase2, firstKeyIndex);

  k1 = reinterpret_cast<PoseKeyFrame*>(kBase1);

  if (gz::math::equal(t, 0.0))
  {
    _kf.Translation(k1->Translation());
    _kf.Rotation(k1->Rotation());
  }
  else
  {
    _kf.Translation(this->positionSpline->Interpolate(firstKeyIndex, t));
    _kf.Rotation(this->rotationSpline->Interpolate(firstKeyIndex, t));
  }
}

/////////////////////////////////////////////////
NumericAnimation::NumericAnimation(const std::string &_name,
    const double _length, const bool _loop)
: Animation(_name, _length, _loop)
{
}

/////////////////////////////////////////////////
NumericAnimation::~NumericAnimation()
{
}

/////////////////////////////////////////////////
NumericKeyFrame *NumericAnimation::CreateKeyFrame(const double _time)
{
  NumericKeyFrame *frame = new NumericKeyFrame(_time);
  std::vector<KeyFrame*>::iterator iter =
    std::upper_bound(this->keyFrames.begin(), this->keyFrames.end(),
        reinterpret_cast<KeyFrame*>(frame),
        KeyFrameTimeLess());

  this->keyFrames.insert(iter, frame);
  this->build = true;
  return frame;
}

/////////////////////////////////////////////////
void NumericAnimation::InterpolatedKeyFrame(NumericKeyFrame &_kf) const
{
  // Keyframe pointers
  KeyFrame *kBase1, *kBase2;
  NumericKeyFrame *k1, *k2;
  unsigned int firstKeyIndex;

  double t;
  t = this->KeyFramesAtTime(this->timePos, &kBase1, &kBase2, firstKeyIndex);

  k1 = reinterpret_cast<NumericKeyFrame*>(kBase1);
  k2 = reinterpret_cast<NumericKeyFrame*>(kBase2);

  if (gz::math::equal(t, 0.0))
  {
    // Just use k1
    _kf.Value(k1->Value());
  }
  else
  {
    // Interpolate by t
    double diff = k2->Value() - k1->Value();
    _kf.Value(k1->Value() + diff * t);
  }
}