# CN7 맵 Codex 작업 규칙

- 너트 체결 구체화는 `main`의 `map_fasten/`에서 작업한다. 다른 공정과 공용 파일을 수정할 때는 변경 범위를 먼저 확인한다.
- 요청의 목표와 완료 기준을 먼저 잡고 관련 파일부터 읽는다. `rg -n`으로 위치를 찾고 필요한 줄만 확인한다.
- 빌드·Isaac Sim 전체 로그는 임시 파일에 저장한다. 종료 코드, 판정 줄, 오류 전후의 필요한 줄만 읽고 원인 확인에 따라 범위를 넓힌다.
- 변경과 관련된 검증을 먼저 실행한다. 최종 답변에는 변경점, 확인한 결과, 사용자가 바로 쓸 명령을 간결하게 적는다.
- 체결 시험 명령과 GUI 조작법은 [실행 안내](map_fasten/README.md)를 참고한다.
- 커밋 제목은 `feat: <한국어 문장>`, `docs: <한국어 문장>`, `fix: <한국어 문장>` 중 변경에 맞는 형식으로 쓴다. 예시와 분류 기준은 [루트 README](README.md#커밋-메시지)를 참고한다.

독립 작업은 새 대화에서 목표·관련 경로·완료 기준을 짧게 전달한다. 같은 작업의 대화가 길어졌다면 Codex CLI에서 `/compact`로 다음 턴에 전달할 대화 맥락을 줄인다.

[OpenAI의 간결한 지침 작성 가이드](https://developers.openai.com/blog/rethinking-skills-and-prompts-for-gpt-6-astra) · [AGENTS.md 적용 범위](https://learn.chatgpt.com/docs/agent-configuration/agents-md) · [Codex CLI 명령](https://learn.chatgpt.com/docs/developer-commands?surface=cli)
