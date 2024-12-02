# 기여 가이드라인

## 프로젝트에 기여하기
이 프로젝트에 기여해 주셔서 감사합니다! 기여자들을 위한 가이드라인입니다.

### 기여 과정
1. 이 저장소를 Fork 합니다
2. 새로운 Branch를 만듭니다 (`git checkout -b feature/amazing-feature`)
3. 변경사항을 Commit 합니다 (`git commit -m 'Add amazing feature'`)
4. Branch에 Push 합니다 (`git push origin feature/amazing-feature`)
5. Pull Request를 생성합니다

### Pull Request 가이드라인
- PR을 생성하기 전에 최신 main branch와 동기화해주세요
- 명확한 PR 설명을 작성해주세요
- 가능한 경우 테스트를 추가해주세요
- 코드 스타일 가이드를 준수해주세요

### 이슈 보고
버그를 발견하셨나요? 새로운 기능을 제안하고 싶으신가요?
- 먼저 같은 이슈가 이미 있는지 확인해주세요
- 이슈 템플릿을 사용해 상세히 설명해주세요
- 버그 보고 시 재현 방법을 포함해주세요

### 개발 환경 설정
```bash
# 의존성 설치
sudo apt-get install ros-humble-desktop
pip install -r requirements.txt

# 빌드
colcon build

# 테스트 실행
colcon test
```

### 코드 스타일
- ROS2 코딩 스타일을 따릅니다
- 함수와 클래스에 문서화 주석을 추가해주세요
- 변수명은 명확하고 이해하기 쉽게 작성해주세요

### 커밋 메시지 가이드라인
```
feat: 새로운 기능 추가
fix: 버그 수정
docs: 문서 수정
style: 코드 포맷팅
refactor: 코드 리팩토링
test: 테스트 코드 추가
chore: 빌드 작업 업데이트
```
